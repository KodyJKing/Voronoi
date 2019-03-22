class Vector {
    constructor(x, y) {
        this.x = x
        this.y = y
    }
    get length() { return Math.sqrt(this.x * this.x + this.y * this.y) }
    get lengthSquared() { return this.x * this.x + this.y * this.y }
    get unit() { return this.multiply(1 / this.length) }
    get leftNormal() { return new Vector(-this.y, this.x) }
    get rightNormal() { return new Vector(this.y, -this.x) }
    get angle() { return Math.atan2(this.y, this.x) }
    get negate() { return new Vector(-this.x, -this.y) }
    add(other) { return new Vector(this.x + other.x, this.y + other.y) }
    subtract(other) { return new Vector(this.x - other.x, this.y - other.y) }
    dot(other) { return this.x * other.x + this.y * other.y }
    cross(other) { return this.x * other.y - this.y * other.x }
    multiply(scale) { return new Vector(this.x * scale, this.y * scale) }
    lerp(other, t) { return this.multiply(1 - t).add(other.multiply(t)) }
    static polar(angle, length) {
        return new Vector(Math.cos(angle) * length, Math.sin(angle) * length)
    }
    static random(length) {
        let angle = Math.random() * 2 * Math.PI
        return Vector.polar(angle, length)
    }
}

class Line {
    constructor(point, heading, normal = null) {
        this.point = point
        this.heading = heading
        this.forward = Infinity
        this.backward = -Infinity
        this.normal = normal
    }

    timeToIntersection(other) {
        let n = other.heading.leftNormal
        return other.point.subtract(this.point).dot(n) / this.heading.dot(n)
    }

    pointOfIntersection(other) {
        let t = this.timeToIntersection(other)
        if (Number.isFinite(t))
            return this.point.add(this.heading.multiply(t))
        return null
    }

    clip(point, normal) {
        let projection = this.heading.dot(point.subtract(this.point))
        let forward = this.heading.dot(normal) >= 0
        if (forward)
            this.forward = Math.min(this.forward, projection)
        else
            this.backward = Math.max(this.backward, projection)
    }

    get fullyClipped() {
        return this.forward < this.backward
    }

    outerNormal(innerPoint) {
        if (this.normal != null)
            return this.normal
        let normal = this.heading.rightNormal
        let negate = this.point.subtract(innerPoint).dot(normal) < 0
        return negate ? normal.negate : normal
    }

    get forwardPoint() {
        return this.point.add(this.heading.multiply(this.forward))
    }

    get backwardPoint() {
        return this.point.add(this.heading.multiply(this.backward))
    }
}

class Polygon {
    constructor(point) {
        this.point = point
        this.lines = new Set()
    }

    addLine(line) {
        for (let otherLine of this.lines)
            this.clip(line, otherLine)
        this.lines.add(line)
        this.cleanup()
    }

    clip(a, b) {
        let point = a.pointOfIntersection(b)
        if (point == null)
            return
        a.clip(point, b.outerNormal(this.point))
        b.clip(point, a.outerNormal(this.point))
    }

    cleanup() {
        for (let line of this.lines)
            if (line.fullyClipped)
                this.lines.delete(line)
    }

    get sortedPoints() {
        let points = []
        for (let line of this.lines) {
            points.push(line.forwardPoint)
            points.push(line.backwardPoint)
        }
        let average = points.reduce((a, b) => a.add(b), new Vector(0, 0)).multiply(1 / points.length)
        points.sort((a, b) => a.subtract(average).angle - b.subtract(average).angle)
        let halfPoints = [] // If the shape is closed, every point occured twice.
        for (let i = 0; i < points.length; i += 2)
            halfPoints.push(points[i])
        return halfPoints
    }
}

function area(vertices) {
    let area = 0
    for (let i = 0; i < vertices.length; i++) {
        let p0 = vertices[i]
        let p1 = vertices[(i + 1) % vertices.length]
        area += (p1.y - p0.y) * (p1.x + p0.x) / 2
    }
    return area
}

// Calculates center of mass by decomposing polygon into trapezoids.
// This is like the shoelace method but for center of mass: https://jrkoenig.com/2016/10/20/a-geometric-derivation-of-the-shoelace-theorem/
// The center of mass is computed as (∫∫ y dA) / area and (∫∫ x dA) / area.
// Each line segment forms a trapezoid with the y axis.
// For each trapezoid, the area and integral of y over the area are computed.
// When y is decreasing, area and the y integral are negated,
// effectively removing that trapezoid rather than adding it.
// Similar logic is used for x.
function centerOfMass(vertices) {
    // cy * area = ∫∫ y dA = ∫ y x(y) dy = ∫ y (x0 + m (y - y0)) dy
    // = ∫ x0 y - m y0 y + y^2 dy = ∫ (x0 - m y0) y + y^2 dy
    // = 1/2 y^2 (x0 - m y0) + 1/3 m y^3
    let integral = (y, m, x0, y0) => (m * (y ** 3) / 3) + ((x0 - m * y0) * (y ** 2) / 2)

    let area = 0
    let cxa = 0 // Center of mass' x times area
    let cya = 0 // Center of mass' y times area
    for (let i = 0; i < vertices.length; i++) {
        let p0 = vertices[i]
        let p1 = vertices[(i + 1) % vertices.length]
        let y0 = p0.y
        let x0 = p0.x
        let y1 = p1.y
        let x1 = p1.x
        let dy = y1 - y0
        let dx = x1 - x0

        // Add signed area of trapezoid formed between line segment and the y-axis
        area += dy * (x1 + x0) / 2

        if (Math.abs(dy) > 0.001) {
            cya += integral(y1, dx / dy, x0, y0)
            cya -= integral(y0, dx / dy, x0, y0)
        }

        if (Math.abs(dx) > 0.001) {
            // For x, use the reciprocal of the slope and reverse the direction.
            cxa += integral(x0, dy / dx, y1, x1)
            cxa -= integral(x1, dy / dx, y1, x1)
        }
    }
    return new Vector(cxa / area, cya / area)
}
class VoronoiDiagram {

    constructor(points, boundaryPoints) {
        this.polygons = []
        let added = new Set()
        for (let point of points) {
            let str = JSON.stringify(point)
            if (!added.has(str)) {
                added.add(str)
                this.polygons.push(new Polygon(point))
            }
        }

        this.addPairBoundries()
        this.clipToBounds(boundaryPoints)
    }

    addPairBoundries() {
        for (let i = 0; i < this.polygons.length; i++) {
            for (let j = i + 1; j < this.polygons.length; j++) {
                let pli = this.polygons[i]
                let plj = this.polygons[j]
                let pi = pli.point
                let pj = plj.point
                let midPoint = pi.lerp(pj, 0.5)
                let heading = pj.subtract(pi).rightNormal.unit
                let line = new Line(midPoint, heading)
                pli.addLine(line)
                plj.addLine(line)
            }
        }
    }

    clipToBounds(points) {
        for (let i = 0; i < points.length; i++) {
            let j = (i + 1) % points.length
            let pi = points[i]
            let pj = points[j]
            let heading = pj.subtract(pi).unit
            let normal = heading.rightNormal
            for (let polygon of this.polygons)
                polygon.addLine(new Line(pi, heading, normal))
        }
    }
}