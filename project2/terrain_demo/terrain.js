class Noise {
    constructor(seed = 0) {
        this.Grad = class {
            constructor(x, y, z) {
                this.x = x;
                this.y = y;
                this.z = z;
            }
            dot2(x, y) {
                return this.x * x + this.y * y;
            }
            dot3(x, y, z) {
                return this.x * x + this.y * y + this.z * z;
            }
        };

        this.grad3 = [
            new this.Grad(1, 1, 0), new this.Grad(-1, 1, 0),
            new this.Grad(1, -1, 0), new this.Grad(-1, -1, 0),
            new this.Grad(1, 0, 1), new this.Grad(-1, 0, 1),
            new this.Grad(1, 0, -1), new this.Grad(-1, 0, -1),
            new this.Grad(0, 1, 1), new this.Grad(0, -1, 1),
            new this.Grad(0, 1, -1), new this.Grad(0, -1, -1)
        ];

        this.p = [151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180];
        
        this.perm = new Array(512);
        this.gradP = new Array(512);

        this.F2 = 0.5 * (Math.sqrt(3) - 1);
        this.G2 = (3 - Math.sqrt(3)) / 6;
        this.F3 = 1 / 3;
        this.G3 = 1 / 6;

        this.seed(seed);
    }

    seed(seed) {
        if (seed > 0 && seed < 1) {
            seed *= 65536;
        }
        seed = Math.floor(seed);
        if (seed < 256) {
            seed |= seed << 8;
        }
        for (let i = 0; i < 256; i++) {
            let v;
            if (i & 1) {
                v = this.p[i] ^ (seed & 255);
            } else {
                v = this.p[i] ^ ((seed >> 8) & 255);
            }
            this.perm[i] = this.perm[i + 256] = v;
            this.gradP[i] = this.gradP[i + 256] = this.grad3[v % 12];
        }
    }

    simplex2(xin, yin) {
        let n0, n1, n2;
        const s = (xin + yin) * this.F2;
        let i = Math.floor(xin + s);
        let j = Math.floor(yin + s);
        const t = (i + j) * this.G2;
        const x0 = xin - i + t;
        const y0 = yin - j + t;

        let i1, j1;
        if (x0 > y0) {
            i1 = 1; j1 = 0;
        } else {
            i1 = 0; j1 = 1;
        }

        const x1 = x0 - i1 + this.G2;
        const y1 = y0 - j1 + this.G2;
        const x2 = x0 - 1 + 2 * this.G2;
        const y2 = y0 - 1 + 2 * this.G2;

        i &= 255;
        j &= 255;
        const gi0 = this.gradP[i + this.perm[j]];
        const gi1 = this.gradP[i + i1 + this.perm[j + j1]];
        const gi2 = this.gradP[i + 1 + this.perm[j + 1]];

        let t0 = 0.5 - x0 * x0 - y0 * y0;
        if (t0 < 0) {
            n0 = 0;
        } else {
            t0 *= t0;
            n0 = t0 * t0 * gi0.dot2(x0, y0);
        }

        let t1 = 0.5 - x1 * x1 - y1 * y1;
        if (t1 < 0) {
            n1 = 0;
        } else {
            t1 *= t1;
            n1 = t1 * t1 * gi1.dot2(x1, y1);
        }

        let t2 = 0.5 - x2 * x2 - y2 * y2;
        if (t2 < 0) {
            n2 = 0;
        } else {
            t2 *= t2;
            n2 = t2 * t2 * gi2.dot2(x2, y2);
        }

        return 70 * (n0 + n1 + n2);
    }
}

class CelestialBody {
    constructor(type, radius, position, color) {
        this.type = type;
        this.radius = radius;
        this.position = position;
        this.color = color;
        this.vertices = [];
        this.colors = [];
        this.noise = new Noise(Date.now());
    }

    generateSphere(detail = 20) {
        this.vertices = [];  // Clear existing vertices
        let triangleVertices = [];
        let triangleColors = [];
        
        // Generate sphere vertices using UV sphere method
        for (let lat = 0; lat <= detail; lat++) {
            const theta = lat * Math.PI / detail;
            const sinTheta = Math.sin(theta);
            const cosTheta = Math.cos(theta);
    
            for (let lon = 0; lon <= detail; lon++) {
                const phi = lon * 2 * Math.PI / detail;
                const sinPhi = Math.sin(phi);
                const cosPhi = Math.cos(phi);
    
                // Add noise to radius for rocky planets
                let radius = this.radius;
                let noiseValue = 0;
                if (this.type === 'rocky') {
                    noiseValue = this.noise.simplex2(lat/detail * 5, lon/detail * 5) * 0.1;
                    radius += radius * noiseValue;
                }
    
                const x = cosPhi * sinTheta;
                const y = cosTheta;
                const z = sinPhi * sinTheta;
    
                this.vertices.push(
                    this.position[0] + x * radius,
                    this.position[1] + y * radius,
                    this.position[2] + z * radius
                );
            }
        }
    
        // Generate indices for triangles
        let indices = [];
        for (let lat = 0; lat < detail; lat++) {
            for (let lon = 0; lon < detail; lon++) {
                const first = (lat * (detail + 1)) + lon;
                const second = first + detail + 1;
    
                indices.push(first, second, first + 1);
                indices.push(second, second + 1, first + 1);
            }
        }
    
        // Convert indices to vertices and generate colors
        for (let i = 0; i < indices.length; i++) {
            const idx = indices[i] * 3;
            triangleVertices.push(
                this.vertices[idx],
                this.vertices[idx + 1],
                this.vertices[idx + 2]
            );
    
            // Generate color based on height and noise for rocky planets
            let colorMod = 1.0;
            if (this.type === 'rocky') {
                // Use y-coordinate (height) and noise to modify color
                const height = this.vertices[idx + 1];
                const noiseValue = this.noise.simplex2(height * 2, idx * 0.1) * 0.5 + 0.5;
                colorMod = 0.7 + noiseValue * 0.3;
            }
    
            // Add colors
            triangleColors.push(
                this.color[0] * colorMod,
                this.color[1] * colorMod,
                this.color[2] * colorMod
            );
        }
    
        this.vertices = triangleVertices;
        this.colors = triangleColors;
        
        return triangleVertices;
    }
}

class SpaceGenerator {
    constructor() {
        this.noise = new Noise(Date.now());
    }

    // Rest of the SpaceGenerator class remains the same
    generateStar(position, brightness = 1.0) {
        return {
            position: position,
            color: [brightness, brightness, brightness],
            size: 0.01 + Math.random() * 0.02
        };
    }

    generateStarfield(count, bounds) {
        let stars = [];
        for (let i = 0; i < count; i++) {
            const position = [
                (Math.random() - 0.5) * bounds * 2,
                (Math.random() - 0.5) * bounds * 2,
                (Math.random() - 0.5) * bounds * 2
            ];
            const brightness = 0.5 + Math.random() * 0.5;
            stars.push(this.generateStar(position, brightness));
        }
        return stars;
    }

    generatePlanet(position, radius, type) {
        let baseColor;
        let noiseScale;
        let colorVariation;

        switch (type) {
            case 'rocky':
                baseColor = [0.8, 0.6, 0.5];
                noiseScale = 2.0;
                colorVariation = 0.3;
                break;
            case 'gas':
                baseColor = [0.6, 0.7, 0.9];
                noiseScale = 1.0;
                colorVariation = 0.2;
                break;
            case 'ice':
                baseColor = [0.8, 0.9, 1.0];
                noiseScale = 1.5;
                colorVariation = 0.1;
                break;
            default:
                baseColor = [0.7, 0.7, 0.7];
                noiseScale = 1.0;
                colorVariation = 0.2;
        }

        return new CelestialBody('planet', radius, position, baseColor);
    }

    generateSun(position, radius) {
        return new CelestialBody('sun', radius, position, [1.0, 0.9, 0.2]);
    }
}