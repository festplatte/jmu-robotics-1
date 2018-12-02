const mathjs = require("mathjs");
const kParams = require("./rhinoXR3");

module.exports = {
  // Task a)
  calcToolConfigurationVector = (q) => {
    const c1 = mathjs.cos(q[0]);
    const c2 = mathjs.cos(q[1]);
    const c23 = c2 * mathjs.cos(q[2]);
    const c234 = c23 * mathjs.cos(q[3]);
    const s1 = mathjs.sin(q[0]);
    const s2 = mathjs.sin(q[1]);
    const s23 = s2 * mathjs.sin(q[2]);
    const s234 = s23 * mathjs.sin(q[3]);
    const exp5 = mathjs.exp(q[4] / mathjs.pi);
    const temp1 =
      kParams.a[1] * c2 +
      kParams.a[2] * c23 +
      kParams.a[3] * c234 -
      kParams.d[4] * s234;

    return [
      c1 * temp1,
      s1 * temp1,
      kParams.d[0] -
        kParams.a[1] * s2 -
        kParams.a[2] * s23 -
        kParams.a[3] * s234 -
        kParams.d[4] * c234,
      -exp5 * c1 * s234,
      -exp5 * s1 * s234,
      -exp5 * c234
    ];
  },

  // Task b)
  calcJointVariables = (w) => {
    const q1 = mathjs.atan2(w[1], w[0]);
    const q234 = mathjs.atan2(
      -(mathjs.cos(q1) * w[3] + mathjs.sin(q1) * w[4]),
      -w[5]
    );
    const s234 = mathjs.sin(q234);
    const c234 = mathjs.cos(q234);
    const b1 =
      mathjs.cos(q1) * w[0] +
      mathjs.sin(q1) * w[1] -
      kParams.a[3] * c234 +
      kParams.d[4] * s234;
    const b2 = kParams.d[0] - w[2] - kParams.a[3] * s234 - kParams.d[4] * c234;
    const bSquare = mathjs.square(b1) + mathjs.square(b2);
    const c3 =
      (bSquare - mathjs.square(kParams.a[1]) - mathjs.square(kParams.a[2])) /
      (2 * kParams.a[1] * kParams.a[2]);
    const q3 = mathjs.acos(c3);
    const s3 = mathjs.sin(q3);
    const q2 = mathjs.atan2(
      (kParams.a[1] + kParams.a[2] * c3) * b2 - kParams.a[3] * s3 * b1,
      (kParams.a[1] + kParams.a[2] * c3) * b1 + kParams.a[3] * s3 * b2
    );
    const q4 = q234 - q3 - q2;
    const q5 =
      mathjs.pi *
      mathjs.log(
        mathjs.sqrt(
          mathjs.square(w[3]) + mathjs.square(w[4]) + mathjs.square(w[5])
        ),
        mathjs.exp(1)
      );

    return [q1, q2, q3, q4, q5];
  },

  // Task c)
  calcVelocityJointVariables = (w, dw, q) => {
    const dq1 =
      (w[0] * dw[1] - w[1] * dw[0]) / (mathjs.square(w[0]) + mathjs.square(w[1]));

    const c1 = mathjs.cos(q[0]);
    const s1 = mathjs.sin(q[0]);
    const c3 = mathjs.cos(q[2]);
    const s3 = mathjs.sin(q[2]);

    const q234 = q[1] + q[2] + q[3];
    const c234 = mathjs.cos(q234);
    const s234 = mathjs.sin(q234);

    const b0 = c1 * w[3] + s1 * w[4];
    const b1 = c1 * w[0] + s1 * w[1] - kParams.a[3] * c234 + kParams.d[4] * s234;
    const b2 = kParams.d[0] - w[2] - kParams.a[3] * s234 - kParams.d[4] * c234;
    const db0 = c1 * dw[3] + s1 * dw[4] + (c1 * w[4] - s1 * w[3]) * dq1;
    const dq234 =
      (db0 * w[5] - b0 * dw[5]) / (mathjs.square(b0) + mathjs.square(w[5]));

    const db1 =
      c1 * dw[0] +
      s1 * dw[1] +
      (c1 * w[1] - s1 * w[0]) * dq1 +
      (kParams.a[3] * s234 + kParams.d[4] * c234) * dq234;

    const db2 = (kParams.d[4] * s234 - kParams.a[3] * c234) * dq234 * dw[2];

    const dq3 =
      (2 * (b1 * db1 + b2 * db2)) /
      mathjs.sqrt(
        mathjs.square(2 * kParams.a[1] * kParams.a[2]) -
          mathjs.square(
            mathjs.square(b1) +
              mathjs.square(b2) -
              mathjs.square(kParams.a[1]) -
              mathjs.square(kParams.a[2])
          )
      );

    const b3 = (kParams.a[1] + kParams.a[2] * c3) * b1 + kParams.a[2] * s3 * b2;
    const b4 = (kParams.a[1] + kParams.a[2] * c3) * b2 - kParams.a[2] * s3 * b1;
    const db3 =
      (kParams.a[1] + kParams.a[2] * c3) * db1 +
      kParams.a[2] * s3 * db2 +
      kParams.a[2] * (c3 * b2 - s3 * b1) * dq3;
    const db4 =
      (kParams.a[1] + kParams.a[2] * c3) * db2 -
      kParams.a[2] * s3 * db1 -
      kParams.a[2] * (c3 * b1 - s3 * b2) * dq3;
    const dq2 = (b3 * db4 - b4 * db3) / (mathjs.square(b3) + mathjs.square(b4));

    const dq4 = dq234 - dq2 - dq3;

    const dq5 =
      (mathjs.pi * (w[3] * dw[3] + w[4] * dw[4] + w[5] * dw[5])) /
      (mathjs.square(w[3]) + mathjs.square(w[4]) + mathjs.square(w[5]));

    return [dq1, dq2, dq3, dq4, dq5];
  },

  // Task d)
  speedDist = (t, T, tau) => {
    if (t <= tau) {
      return mathjs.square(t / T);
    }
    if (t < T - tau) {
      return (
        (1 - this.speedDist(T - tau, T, tau)) * (t - tau / T - 2 * tau) +
        this.speedDist(tau, T, tau)
      );
    }
    if (t <= T) {
      return 1 - mathjs.square((T - t) / T);
    }
    return 1;
  }
}
