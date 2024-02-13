// struct Pid {
//     kp: f32,
//     ki: f32,
//     kd: f32,
//     integral: f32,
//     prev_error: f32,
// }
// impl Pid {
//     fn new(kp: f32, ki: f32, kd: f32) -> Pid {
//         Pid {
//             kp,
//             ki,
//             kd,
//             integral: 0.0,
//             prev_error: 0.0,
//         }
//     }
//
//     fn update(&mut self, s_t: f32, p_t: f32, dt: f32) -> f32 {
//         let e = s_t - p_t;
//
//         let p = self.kp * e;
//
//         self.integral += e * dt;
//         let i = self.ki * self.integral;
//
//         let mut d = (e - self.prev_error) / dt;
//         d *= self.kd;
//
//         self.prev_error = e;
//         return p + i + d;
//     }
// }
