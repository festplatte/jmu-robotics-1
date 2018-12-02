const kCalc = require("./kinematicsCalculator");
const mathjs = require("mathjs");

// Task c) TODO
let values = [
    [0, -120, 70, 0, -90],
    [45, -135, 0, 0, 0],
    [180, -122.5, 45, -120, 90]
]

values.forEach(vals => {
    let radVals = vals.map(x => kCalc.transformRadToDeg(x));
    let armMatrix = kCalc.armMatrix(radVals[0], radVals[1], radVals[2], radVals[3], radVals[4]);

    console.log(`Arm matrix for following values: ${vals}`);
    console.log(`   corresponding rad values: ${radVals}`);
    console.log(mathjs.round(armMatrix, 4));
    console.log();
    console.log();
    console.log();
});