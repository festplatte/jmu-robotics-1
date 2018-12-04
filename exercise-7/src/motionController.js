const mathjs = require("mathjs");
const kCalc = require("./kinematicsCalculator");

const deltaT = 0.1;

module.exports = function(w0, w1, T) {
  const tau = T * 0.1;
  const q0 = kCalc.calcJointVariables(w0);

  let currentQ = q0;
  let currentW = w0;

  for (let t = 0; t <= T; t += deltaT) {
    const s = kCalc.speedDist(t, T, tau);
    const wd = mathjs.add(mathjs.multiply(w0, 1 - s), mathjs.multiply(w1, s));
    const dwd = mathjs.divide(mathjs.subtract(wd, currentW), deltaT);
    const dqd = kCalc.calcVelocityJointVariables(currentW, dwd, currentQ);
    currentQ = mathjs.add(currentQ, mathjs.multiply(dqd, deltaT));
    currentW = kCalc.calcToolConfigurationVector(currentQ);

    console.log(`t: ${t}, s: ${s}`);
    console.log(`wd: ${wd}`);
    console.log(`dwd: ${dwd}`);
    console.log(`dqd: ${dqd}`);
    console.log(`q: ${q}`);
    console.log(`w: ${w}`);
    console.log(`wd - w: ${mathjs.subtract(wd, w)}`);
  }
};
