import { vec3 } from "./geometry";
import { Robot } from "./robot";
import { RotationalJoint, SlidingJoint, StaticJoint } from "./joint";

export function robotToUrdf(robot: Robot): string {
  // https://wiki.ros.org/urdf/XML/link
  let links = robot.links
    .map((link, i) => {
      let normal = vec3(0, 0, 1).applyQuaternion(link.quaternion);
      let rpy = `${Math.asin(-normal.y)} ${Math.atan2(normal.x, normal.z)} 0`;
      let xyz = `${link.position.x / 1000} ${link.position.y / 1000} ${link.position.z / 1000}`;

      let geometry =
        typeof link.width !== "undefined" &&
        typeof link.height !== "undefined" &&
        typeof link.depth !== "undefined"
          ? `<geometry>
    <box size="${link.width / 1000} ${link.height / 1000} ${link.depth / 1000}" />
  </geometry>`
          : "";

      return `<link name="${i}">
<visual>
  <origin xyz="${xyz}" rpy="${rpy}"/>
  ${geometry} 
</visual>
<collision>
  <origin xyz="${xyz}" rpy="${rpy}"/>
  ${geometry} 
</collision>
</link>`;
    })
    .join("\n");

  // https://wiki.ros.org/urdf/XML/joint
  let joints = robot.joints
    .map((joint) => {
      let normal = vec3(0, 0, 1).applyQuaternion(joint.quaternion);
      let rpy = `${Math.asin(-normal.y)} ${Math.atan2(normal.x, normal.z)} 0`;
      let xyz = `${joint.position.x / 1000} ${joint.position.y / 1000} ${joint.position.z / 1000}`;
      console.log(xyz);

      if (joint instanceof RotationalJoint) {
        return `<joint name="${joint.name}" type="revolute">
  <origin xyz="${xyz}" rpy="${rpy}"/>
  <parent link="${joint.base.name}" />
  <child link="${joint.attachment.name}" />
  <axis xyz="${joint.axis.x} ${joint.axis.y} ${joint.axis.z}" />
  <limit lower="${joint.min}" upper="${joint.max}" effort="0" velocity="${joint.maxVelocity}"/>
</joint>`;
      } else if (joint instanceof SlidingJoint) {
        return `<joint name="${joint.name}" type="prismatic">
  <origin xyz="${xyz}" rpy="${rpy}"/>
  <parent link="${joint.base.name}" />
  <child link="${joint.attachment.name}" />
  <axis xyz="${joint.axis.x} ${joint.axis.y} ${joint.axis.z}" />
  <limit lower="${joint.min}" upper="${joint.max}" effort="0" velocity="${joint.maxVelocity}"/>
</joint>`;
      } else if (joint instanceof StaticJoint) {
        return `<joint name="${joint.name}" type="fixed">
  <origin xyz="${xyz}" rpy="${rpy}"/>
  <parent link="${joint.base.name}" />
  <child link="${joint.attachment.name}" />
</joint>`;
      }

      throw Error(`invalid joint type ${joint.type}`);
    })
    .join("\n");

  return `<robot>
${links}

${joints}
</robot>`;
}
