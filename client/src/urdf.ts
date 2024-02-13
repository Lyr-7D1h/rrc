import { vec3 } from './geometry'
import { type Robot } from './robot'
import { RevoluteJoint, PrismaticJoint, FixedJoint } from './joint'

export function robotToUrdf(robot: Robot): string {
  // https://wiki.ros.org/urdf/XML/link
  const links = robot.links
    .map((link, i) => {
      const normal = vec3(0, 0, 1).applyQuaternion(link.quaternion)
      const rpy = `${Math.asin(-normal.y)} ${Math.atan2(normal.x, normal.z)} 0`
      const xyz = `${link.position.x / 1000} ${link.position.y / 1000} ${link.position.z / 1000}`

      const geometry =
        typeof link.width !== 'undefined' &&
        typeof link.height !== 'undefined' &&
        typeof link.depth !== 'undefined'
          ? `<geometry>
    <box size="${link.width / 1000} ${link.height / 1000} ${link.depth / 1000}" />
  </geometry>`
          : ''

      return `<link name="${i}">
<visual>
  <origin xyz="${xyz}" rpy="${rpy}"/>
  ${geometry} 
</visual>
${
  typeof link.depth !== 'undefined'
    ? `<collision>
  <origin xyz="${xyz}" rpy="${rpy}"/>
  ${geometry} 
</collision>`
    : ''
}
</link>`
    })
    .join('\n')

  // https://wiki.ros.org/urdf/XML/joint
  const joints = robot.joints
    .map((joint) => {
      const parent = robot.links.find((l) => l.name === joint.base.name)!

      const normal = vec3(0, 0, 1)
        .applyQuaternion(parent.quaternion)
        .applyQuaternion(joint.quaternion)
      const rpy = `${Math.asin(-normal.y)} ${Math.atan2(normal.x, normal.z)} 0`

      // get the joint position relative to the joint position of the parent
      const position = joint.position
        .clone()
        .applyQuaternion(parent.quaternion)
        .add(parent.position)
      const xyz = `${position.x / 1000} ${position.y / 1000} ${position.z / 1000}`

      if (joint instanceof RevoluteJoint) {
        // from degrees to rad
        const lower = (joint.min * 180) / Math.PI
        const upper = (joint.max * 180) / Math.PI
        const velocity = (joint.maxVelocity * 180) / Math.PI
        return `<joint name="${joint.name}" type="revolute">
  <origin xyz="${xyz}" rpy="${rpy}"/>
  <parent link="${joint.base.name}" />
  <child link="${joint.attachment.name}" />
  <axis xyz="${joint.axis.x} ${joint.axis.y} ${joint.axis.z}" />
  <limit lower="${lower}" upper="${upper}" effort="100" velocity="${velocity}"/>
</joint>`
      } else if (joint instanceof PrismaticJoint) {
        // from mm to m
        const lower = joint.min / 1000
        const upper = joint.max / 1000
        const velocity = joint.maxVelocity / 1000
        return `<joint name="${joint.name}" type="prismatic">
  <origin xyz="${xyz}" rpy="${rpy}"/>
  <parent link="${joint.base.name}" />
  <child link="${joint.attachment.name}" />
  <axis xyz="${joint.axis.x} ${joint.axis.y} ${joint.axis.z}" />
  <limit lower="${lower}" upper="${upper}" effort="100" velocity="${velocity}"/>
</joint>`
      } else if (joint instanceof FixedJoint) {
        return `<joint name="${joint.name}" type="fixed">
  <origin xyz="${xyz}" rpy="${rpy}"/>
  <parent link="${joint.base.name}" />
  <child link="${joint.attachment.name}" />
</joint>`
      }

      throw Error(`invalid joint type ${joint.type}`)
    })
    .join('\n')

  return `<robot>
${links}

${joints}
</robot>`
}
