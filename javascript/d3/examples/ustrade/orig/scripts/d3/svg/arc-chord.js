/**

import "../core/functor";
import "../core/source";
import "../core/target";
import "../math/trigonometry";
import "arc";
import "svg";

*/

function f_yOffsetFactor(d) {
    return d.yOffsetFactor;
}

d3.svg.arc_chord = function() {
    var source = d3_source,
        target = d3_target,
        radius = d3_svg_chordRadius,
        startAngle = d3_svg_arcStartAngle,
        endAngle = d3_svg_arcEndAngle,
        yOffsetFactor = f_yOffsetFactor,
        yOffset;

    // TODO Allow control point to be customized.

    function arc_chord(d, i) {
        var s = subgroup(this, source, d, i),
            t = subgroup(this, target, d, i);
            t.p0=[-5, -s.r*(-yOffset/2)]
            t.p1=[5, -s.r*(-yOffset/2)];

        return "M" + s.p0
            + arc(s.r, s.p1, s.a1 - s.a0) + (equals(s, t)
            ? curve(s.r, s.p1, s.r, s.p0)
            : curve(s.r, s.p1, t.r, t.p0, s.a1 - s.a0 )
            + arc(t.r, t.p1, t.a1 - t.a0)
            + curve(t.r, t.p1, s.r, s.p0))
            + "Z";
    }

    function subgroup(self, f, d, i) {
        var subgroup = f.call(self, d, i),
            r = radius.call(self, subgroup, i),
            a0 = startAngle.call(self, subgroup, i) + d3_svg_arcOffset,
            a1 = endAngle.call(self, subgroup, i) + d3_svg_arcOffset;
            yOffset=yOffsetFactor.call(self,subgroup,i);
        return {
            r: r,
            a0: a0,
            a1: a1,
            p0: [r * Math.cos(a0), r * Math.sin(a0)],
            p1: [r * Math.cos(a1), r * Math.sin(a1)]
        };
    }

    function equals(a, b) {
        return a.a0 == b.a0 && a.a1 == b.a1;
    }

    function arc(r, p, a) {
        return " A" + r + "," + r + " 0 " + +(a > π) + ",1 " + p;
    }

    function curve(r0, p0, r1, p1) {
        var r=" Q 0," + (r0 * yOffset) + " " + p1;
        return r;
    }

    arc_chord.yOffsetFactor = function(v) {
        if (!arguments.length) return yOffsetFactor;
        yOffsetFactor = d3_functor(v);
        return arc_chord;
    };

    arc_chord.radius = function(v) {
        if (!arguments.length) return radius;
        radius = d3_functor(v);
        return arc_chord;
    };

    arc_chord.source = function(v) {
        if (!arguments.length) return source;
        source = d3_functor(v);
        return arc_chord;
    };

    arc_chord.target = function(v) {
        if (!arguments.length) return target;
        target = d3_functor(v);
        return arc_chord;
    };

    arc_chord.startAngle = function(v) {
        if (!arguments.length) return startAngle;
        startAngle = d3_functor(v);
        return arc_chord;
    };

    arc_chord.endAngle = function(v) {
        if (!arguments.length) return endAngle;
        endAngle = d3_functor(v);
        return arc_chord;
    };

    return arc_chord;
};

function d3_svg_chordRadius(d) {
    return d.radius;
}
