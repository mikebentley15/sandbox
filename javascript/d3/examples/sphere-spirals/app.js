!function() {
  function t(t, a) {
      n.rotate([t, a, n.rotate()[2]]);
      o.rotate([180 + t, -a, o.rotate()[2]]);
      svg.selectAll("path:not(.back)").attr("d", p);
      svg.selectAll("path.back").attr("d", i);
  }
  function a(t, a) {
      return d3.range(t, t + 3 / a + .005, .01)
               .map(function(e) {
                 return s.invert([(e + 1) % 2 - 1, 1.5 - (e - t) * a]);
               });
  }
  var width = 500;
  var height = width
  var n = d3.geo.orthographic()
            .translate([width / 2, height / 2])
            .scale(height / 2 - 2)
            .clipAngle(90)
            .rotate([0, -30])
  var o = d3.geo.projection(function(t, a) {
              var e = d3.geo.orthographic.raw(t, a);
              e[0] = -e[0];
              return e;
            })
            .translate(n.translate())
            .scale(n.scale())
            .clipAngle(n.clipAngle())
            .rotate([n.rotate()[0] + 180, -n.rotate()[1], -n.rotate()[2]])
  var c = n.rotate()
  var d = [.005, -.001]
  var l = Date.now()
  var p = d3.geo.path().projection(n)
  var i = d3.geo.path().projection(o)
  var svg = d3.select("#map")
            .append("svg")
            .attr("width", width)
            .attr("height", height)
            .call(d3.behavior.drag()
              .origin(function() {
                var t = n.rotate();
                return {
                  x: t[0],
                  y: -t[1]
                }
              })
              .on("drag", function() {
                l = -1,
                t(d3.event.x, -d3.event.y)
              })
              .on("dragend", function() {
                c = n.rotate(),
                l = Date.now()
              })
            );
  var s = d3.geo.mercator().scale(1 / Math.PI).translate([0, 0]);
  var u = 4;
  var h = {
    type: "GeometryCollection",
    geometries: d3.range(-1, 1 + 1 / u, 2 / u).map(function(t) {
      var e = .5;
      return {
        type: "Polygon",
        coordinates: [a(t, e).concat(a(t + 1 / u, e).reverse())]
      }
    })
  };
  svg.append("path").attr("class", "back").datum(h).attr("d", i);
  svg.append("path").attr("class", "front").datum(h).attr("d", p);
  svg.append("path").datum(d3.geo.graticule()).attr("class", "graticule").attr("d", p);
  svg.append("path").datum({
    type: "Sphere"
  }).attr("class", "outline").attr("d", p);
  d3.timer(function() {
    if (!(0 > l)) {
      var a = Date.now() - l;
      t(c[0] + d[0] * a, c[1] + d[1] * a);
    }
  });
}();

