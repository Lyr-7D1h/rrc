# Rustic Robot Controls

Software for simulating, controlling and visualizing robots

> [!WARNING]
> Currently this is a **Work In Progress**. It only simulates the robotic state and does not actually control a robot yet.
> My goal is to eventually use this software to visualize, control and simulate movements of any generic robotic object.

https://github.com/Lyr-7D1h/rrc/assets/23296032/19cc6a0a-c9c9-4313-a62a-65f1c5ae108e

# Usage

**Client**

```
cd client
npm install
npm start
```

**Controls**

```
cd controls
rustup update # ensure latest version of rust
cargo run
```

# Roadmap

- Improve floating point precision
- Offline simulation mode
- Collision detection of self and other objects in space
- Keep camera rotation on dev update
- Improved communication format
    - share error messages and acknowledgements
    - Protobuf/binary communication format
- STL support ([cad models into simulation software](https://github.com/rhoban/onshape-to-robot/)))

# Interesting Resources

**Libraries**
- https://www.npmjs.com/package/urdf-loader
- https://github.com/kripken/ammo.js
- https://sbcode.net/threejs
- https://threejs.org/editor/
- https://robotics.rs/
- https://github.com/openrr/urdf-viz

**Examples**
- https://codepen.io/asterix77/pen/dLdraK
- https://github.com/glumb/robot-gui

**Reading**
- https://ocw.mit.edu/courses/2-12-introduction-to-robotics-fall-2005/c8828a16e71c246b78461dd0596b983f_chapter4.pdf
- http://www.andreasaristidou.com/publications/papers/FABRIK.pdf
- https://courses.shadmehrlab.org/Shortcourse/minimumjerk.pdf (Smooth motion planning)
- https://en.wikipedia.org/wiki/Control_theory
- https://github.com/ericseppanen/epoch_playground
- https://gcc.gnu.org/wiki/Atomic/GCCMM/AtomicSync
- https://eater.net/quaternions
- https://en.wikipedia.org/wiki/Inverse_kinematics
- https://en.wikipedia.org/wiki/Motion_planning
- https://en.wikipedia.org/wiki/Kinematic_chain
- https://en.wikipedia.org/wiki/Holonomic_constraints
- https://en.wikipedia.org/wiki/Kinematic_diagram
- https://en.wikipedia.org/wiki/Mechanical_joint
- https://en.wikipedia.org/wiki/Linkage_(mechanical)
- https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
