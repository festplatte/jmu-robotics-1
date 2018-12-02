const mathjs = require("mathjs");

class kinematicParameters {
    constructor(theta, d, a, alpha){
        this.theta = theta;
        this.d = d;
        this.a = a;
        this.alpha = alpha;
    }
}

module.exports = [
    new kinematicParameters(null, 26.04, 0, -mathjs.pi / 2),
    new kinematicParameters(null, 0, 22.86, 0),
    new kinematicParameters(null, 0, 22.86, 0),
    new kinematicParameters(null, 0, 0.95, -mathjs.pi / 2),
    new kinematicParameters(null, 16.83, 0, 0),
]
