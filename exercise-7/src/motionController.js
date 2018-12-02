const mathjs = require("mathjs");
const kCalc = require("./kinematicsCalculator");

const deltaT = 0.1;

module.exports = function(w0, w1, T) {
  const tau = T * 0.1;
  const q0 = kCalc.calcJointVariables(w0);

  let currentQ = q0;
  let currentW = w0;

  for (let t = 0; t <= T; t += deltaT) {
    const wd = mathjs.multiply(w1, kCalc.speedDist(t, T, tau));
    const wdt = mathjs.divide(mathjs.subtract(newW, currentW), deltaT);
    const qdt = kCalc.calcVelocityJointVariables(currentW, currentQ, wdt);
    // TODO implement rest
  }
};
