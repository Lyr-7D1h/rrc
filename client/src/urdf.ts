import { vec3 } from "./geometry";
import { Robot } from "./robot";
import { RotationalJoint, SlidingJoint, StaticJoint } from "./joint";
import { Link } from "./link";

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
${
  typeof link.depth !== "undefined"
    ? `<collision>
  <origin xyz="${xyz}" rpy="${rpy}"/>
  ${geometry} 
</collision>`
    : ""
}
</link>`;
    })
    .join("\n");

  // https://wiki.ros.org/urdf/XML/joint
  let joints = robot.joints
    .map((joint) => {
      let parent = robot.links.find((l) => l.name === joint.base.name) as Link;

      let normal = vec3(0, 0, 1)
        .applyQuaternion(parent.quaternion)
        .applyQuaternion(joint.quaternion);
      let rpy = `${Math.asin(-normal.y)} ${Math.atan2(normal.x, normal.z)} 0`;

      // get the joint position relative to the joint position of the parent
      let position = joint.position
        .clone()
        .applyQuaternion(parent.quaternion)
        .add(parent.position);
      let xyz = `${position.x / 1000} ${position.y / 1000} ${position.z / 1000}`;

      if (joint instanceof RotationalJoint) {
        // from degrees to rad
        let lower = (joint.min * 180) / Math.PI;
        let upper = (joint.max * 180) / Math.PI;
        let velocity = (joint.maxVelocity * 180) / Math.PI;
        return `<joint name="${joint.name}" type="revolute">
  <origin xyz="${xyz}" rpy="${rpy}"/>
  <parent link="${joint.base.name}" />
  <child link="${joint.attachment.name}" />
  <axis xyz="${joint.axis.x} ${joint.axis.y} ${joint.axis.z}" />
  <limit lower="${lower}" upper="${upper}" effort="100" velocity="${velocity}"/>
</joint>`;
      } else if (joint instanceof SlidingJoint) {
        // from mm to m
        let lower = joint.min / 1000;
        let upper = joint.max / 1000;
        let velocity = joint.maxVelocity / 1000;
        return `<joint name="${joint.name}" type="prismatic">
  <origin xyz="${xyz}" rpy="${rpy}"/>
  <parent link="${joint.base.name}" />
  <child link="${joint.attachment.name}" />
  <axis xyz="${joint.axis.x} ${joint.axis.y} ${joint.axis.z}" />
  <limit lower="${lower}" upper="${upper}" effort="100" velocity="${velocity}"/>
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
