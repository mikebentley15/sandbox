!function() {
    function t(t) {
        return function(e) {
            var n = t(e);
            return -180 === n[0][0] && -90 === n[0][1] && 180 === n[1][0] && 90 === n[1][1] ? {
                type: "Sphere"
            } : (-90 === n[0][1] && (n[0][1] += 1e-6),
            90 === n[1][1] && (n[0][1] -= 1e-6),
            n[0][1] === n[1][1] && (n[1][1] += 1e-6),
            {
                type: "Polygon",
                coordinates: [[n[0]].concat(a(n[1][1], n[0][0], n[1][0])).concat(a(n[0][1], n[0][0], n[1][0]).reverse())]
            })
        }
    }
    function a(t, a, e) {
        a > e && (e += 360);
        var n = e - a
          , o = n / Math.ceil(n);
        return d3.range(a, e + .5 * o, o).map(function(a) {
            return [r(a), t]
        })
    }
    function e(t) {
        var a, e, r, o;
        return r = o = -(a = e = 1 / 0),
        d3.geo.stream(t, {
            point: function(t, n) {
                a > t && (a = t),
                t > r && (r = t),
                e > n && (e = n),
                n > o && (o = n)
            },
            lineStart: n,
            lineEnd: n,
            polygonStart: n,
            polygonEnd: n
        }),
        [[a, e], [r, o]]
    }
    function n() {}
    function r(t) {
        return (t + 180) % 360 - 180
    }
    var width = 600
      , d = width
      , p = d3.geo.orthographic().translate([width / 2, d / 2]).scale(295).clipAngle(90).precision(.1).rotate([0, -30])
      , i = d3.geo.path().projection(p)
      , c = d3.geo.graticule()()
      , s = d3.select("#map").append("svg").attr("width", width).attr("height", d).call(d3.behavior.drag().origin(function() {
        var t = p.rotate();
        return {
            x: 2 * t[0],
            y: -2 * t[1]
        }
    }).on("drag", function() {
        p.rotate([d3.event.x / 2, -d3.event.y / 2, p.rotate()[2]]),
        s.selectAll("path").attr("d", i)
    }))
      , u = s.append("defs").append("pattern").attr("id", "hatch").attr("patternUnits", "userSpaceOnUse").attr("width", 8).attr("height", 8).append("g");
    u.append("path").attr("d", "M0,0L8,8"),
    u.append("path").attr("d", "M8,0L0,8"),
    s.append("path").datum({
        type: "Sphere"
    }).attr("class", "background").attr("d", i),
    s.append("path").datum(c).attr("class", "graticule").attr("d", i),
    s.append("path").datum({
        type: "LineString",
        coordinates: [[180, -90], [180, 0], [180, 90]]
    }).attr("class", "antimeridian").attr("d", i),
    s.append("path").datum({
        type: "Sphere"
    }).attr("class", "graticule").attr("d", i),
    d3.json("../world-110m.json", function(a, e) {
        var n = s.selectAll(".country").data(topojson.feature(e, e.objects.countries).features).enter().append("g").attr("class", "country");
        n.append("path").attr("class", "land").attr("d", i),
        n.append("path").datum(t(d3.geo.bounds)).attr("class", "bounds").attr("d", i)
    }),
    function() {
        var a = 300
          , n = a / 2
          , o = d3.geo.circle()
          , d = d3.geo.equirectangular().translate([a / 2, n / 2]).scale(a / (2 * Math.PI) - 2).precision(.1)
          , p = d3.geo.path().pointRadius(1).projection(d)
          , i = d3.selectAll(".example").data([{
            type: "MultiPoint",
            coordinates: d3.range(20).map(function() {
                return [r(160 + 40 * Math.random()), 45 * Math.random()]
            })
        }, {
            type: "LineString",
            coordinates: [[150, 10], [-150, 0]]
        }, {
            type: "LineString",
            coordinates: [[-45, 45], [45, 45]]
        }, o.origin([180, 0]).angle(150)(), {
            type: "Polygon",
            coordinates: [[[-60, -30], [60, -30], [180, -30], [-60, -30]], [[-60, -60], [180, -60], [60, -60], [-60, -60]]]
        }, {
            type: "Polygon",
            coordinates: [[[-60, -30], [60, -30], [180, -30], [-60, -30]]]
        }]).selectAll("svg").data(function(a) {
            return [e, d3.geo.bounds].map(function(e) {
                return {
                    bounds: t(e)(a),
                    object: a
                }
            })
        }).enter().append("svg").attr("width", a).attr("height", n + 15);
        i.append("text").attr("text-anchor", "middle").attr("transform", "translate(" + [a / 2, n] + ")").attr("dy", "1em").text(function(t, a) {
            return (a ? "Correct" : "Na\xefve 2D") + " Algorithm"
        }),
        i.append("path").datum(function(t) {
            return t.object
        }).attr("class", function(t) {
            return "feature " + t.type
        }).attr("d", p),
        i.append("path").datum(function(t) {
            return t.bounds
        }).attr("class", "bounds").attr("d", p),
        i.append("path").datum(c).attr("class", "graticule").attr("d", p),
        i.append("path").datum({
            type: "Sphere"
        }).attr("class", "outline").attr("d", p),
        d3.select("#inside").append("svg").attr("width", 16).attr("height", 16).append("rect").style("fill", "url(#hatch)").style("stroke", "#000").style("stroke-width", "2px").attr("width", 16).attr("height", 16)
    }()
}();

