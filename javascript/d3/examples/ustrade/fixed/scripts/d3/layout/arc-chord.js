
d3.layout = {arc_chord: null};
d3.layout.arc_chord = function() {
    var arc_chord = {},
        chords,
        groups,
        matrix,
        n,
        padding = 0,
        paddingPercent = 0,
        startAngle= 0,
        endAngle=360,
        sortGroups,
        sortSubgroups,
        yOffsetFactor=0,
        targetChord=d3.svg.arc_chord(),
        sortChords;

    function relayout() {
        var subgroups = {},
            groupSums = [],
            groupIndex = d3.range(n),
            subgroupIndex = [],
            k,
            x,
            x0,
            i,
            j;

        chords = [];
        groups = [];

        // Compute the sum.
        k = 0, i = -1; while (++i < n) {
            x = 0, j = -1; while (++j < n) {
              //  console.log("matrix[i]=" + i);
                x += matrix[i][j];
            }
            groupSums.push(x);
            subgroupIndex.push(d3.range(n));
            k += x;
        }

        // Sort groups…
        if (sortGroups) {
            groupIndex.sort(function(a, b) {
                return sortGroups(groupSums[a], groupSums[b]);
            });
        }

        // Sort subgroups…
        if (sortSubgroups) {
            subgroupIndex.forEach(function(d, i) {
                d.sort(function(a, b) {
                    return sortSubgroups(matrix[i][a], matrix[i][b]);
                });
            });
        }

        var angleDifference=(endAngle-startAngle);
      //  if (angleDifference < -180) angleDifference += 360;
      //  if (angleDifference > 180) angleDifference -= 360;

        // Convert the sum to scaling factor for [0, 2pi].
        // TODO Allow start and end angle to be specified.
        // TODO Allow padding to be specified as percentage?
        k = ((2 * π)*(angleDifference/360) - padding * (n-1)) / k;

        // Compute the start and end angle for each group and subgroup.
        // Note: Opera has a bug reordering object literal properties!
        x = startAngle*d3_radians, i = -1; while (++i < n) {
            x0 = x, j = -1; while (++j < n) {
                var di = groupIndex[i],
                    dj = subgroupIndex[di][j],
                    v = matrix[di][dj],
                    a0 = x,
                    a1 = x += v * k;
                subgroups[di + "-" + dj] = {
                    index: di,
                    subindex: dj,
                    startAngle: a0,
                    endAngle: a1,
                    value: v
                };
            }
            groups[di] = {
                index: di,
                startAngle: x0,
                endAngle: x,
                value: (x - x0) / k
            };
            x += padding;
        }

     //   targetChord.startAngle=175*d3_radians;
     //   targetChord.endAngle=185*d3_radians;
     //   targetChord.radius=100;
        targetChord.yOffsetFactor=yOffsetFactor;



        // Generate chords for each (non-empty) subgroup-subgroup link.
        i = -1; while (++i < n) {
            j = i - 1; while (++j < n) {
                var source = subgroups[i + "-" + j],
                  //  target = subgroups[j + "-" + i];

                    target=targetChord;
                if (source.value || target.value) {
                    chords.push(source.value < target.value
                        ? {source: target, target: source}
                        : {source: source, target: target});
                }
            }
        }

        if (sortChords) resort();
    }

    function resort() {
        chords.sort(function(a, b) {
            return sortChords(
                (a.source.value + a.target.value) / 2,
                (b.source.value + b.target.value) / 2);
        });
    }

    arc_chord.matrix = function(x) {
        if (!arguments.length) return matrix;
        n = (matrix = x) && matrix.length;
        chords = groups = null;
        return arc_chord;
    };

    arc_chord.targetChord = function(x) {
        if (!arguments.length) return targetChord;
        targetChord=x;
        chords = groups = null;
        return targetChord;
    }

    arc_chord.padding = function(x) {
        if (!arguments.length) return padding;
        padding = x;
        chords = groups = null;
        return arc_chord;
    };

    arc_chord.yOffsetFactor = function(x) {
        if (!arguments.length) return yOffsetFactor;
        yOffsetFactor = x;
        chords = groups = null;
        return arc_chord;
    };

    arc_chord.startAngle = function(x) {
        if (!arguments.length) return startAngle;
        startAngle = x;
        chords = groups = null;
        return arc_chord;
    };

    arc_chord.endAngle = function(x) {
        if (!arguments.length) return endAngle;
        endAngle = x;
        chords = groups = null;
        return arc_chord;
    };

    arc_chord.sortGroups = function(x) {
        if (!arguments.length) return sortGroups;
        sortGroups = x;
        chords = groups = null;
        return arc_chord;
    };

    arc_chord.sortSubgroups = function(x) {
        if (!arguments.length) return sortSubgroups;
        sortSubgroups = x;
        chords = null;
        return arc_chord;
    };

    arc_chord.sortChords = function(x) {
        if (!arguments.length) return sortChords;
        sortChords = x;
        if (chords) resort();
        return arc_chord;
    };

    arc_chord.chords = function() {
        if (!chords) relayout();
        return chords;
    };

    arc_chord.groups = function() {
        if (!groups) relayout();
        return groups;
    };



    return arc_chord;
};
