const mathjs = require("mathjs");
const rhinoParameters = require("./rhinoXR3");

/**
 * Task a) calculates the transformation-matrix between the coordinate frames of two joints.
 * @param {Number} theta between -2 * Math.PI and 2 * Math.PI
 * @param {Number} d distance
 * @param {Number} a distance
 * @param {Number} alpha between -2 * Math.PI and 2 * Math.PI
 * @returns {math.Matrix} matrix
 */
function transformationMatrix(theta, d, a, alpha) {
    let sinTheta = mathjs.sin(theta);
    let cosTheta = mathjs.cos(theta);
    let sinAlpha = mathjs.sin(alpha);
    let cosAlpha = mathjs.cos(alpha);

    return mathjs.matrix([
        [cosTheta, -cosAlpha * sinTheta, sinAlpha * sinTheta, a * cosTheta],
        [sinTheta, cosAlpha * cosTheta, -sinAlpha * cosTheta, a * sinTheta],
        [0, sinAlpha, cosAlpha, d],
        [0, 0, 0, 1]
    ]);
}

/**
 * Task b) calculates the arm equation for the rhino XR-3 robot.
 * @param {Number} q1 
 * @param {Number} q2 
 * @param {Number} q3 
 * @param {Number} q4 
 * @param {Number} q5 
 * @returns {math.Matrix} matrix
 */
function armMatrix(q1, q2, q3, q4, q5) {
    if (arguments.length != rhinoParameters.length) {
        throw "Parameters are missing";
    }

    var result = mathjs.identity(4, 4);
    for (var i = 0; i<arguments.length; i++) {
        let q = arguments[i];
        let axis = rhinoParameters[i];
        let m = transformationMatrix(axis.theta != null ? axis.theta : q, axis.d != null ? axis.d : q, axis.a, axis.alpha);
        result = mathjs.multiply(result, m);
    }
    return result;
}

function transformRadToDeg(degrees) {
    return (degrees / 180) * mathjs.pi;
}

module.exports = {
    transformationMatrix: transformationMatrix,
    armMatrix: armMatrix,
    transformRadToDeg: transformRadToDeg
};