
function buildChords(y, m) {

    countries=countriesGrouped[y].values[m].values;

    countries.sort(function (a,b) {
        //Descending Sort
        if (a.Exports > b.Exports) return -1;
        else if (a.Exports < b.Exports) return 1;
        else return 0;
    });

    export_countries = countries.slice(0,topCountryCount);

    countries.sort(function (a,b) {
        //Descending Sort
        if (a.Imports > b.Imports) return -1;
        else if (a.Imports < b.Imports) return 1;
        else return 0;
    });

    import_countries = countries.slice(0,topCountryCount);

    var  import_matrix = [],
        export_matrix = [];

    e_buf_indexByName = e_indexByName;
    i_buf_indexByName = i_indexByName;

    e_indexByName=[];
    e_nameByIndex=[];
    i_indexByName=[];
    i_nameByIndex=[];
    n = 0;

    // Compute a unique index for each package name
    totalExports=0;
    export_countries.forEach(function(d) {
        totalExports += Number(d.Exports);
        d = d.Country;
        if (!(d in e_indexByName)) {
            e_nameByIndex[n] = d;
            e_indexByName[d] = n++;
        }
    });

    export_countries.forEach(function(d) {
        var source = e_indexByName[d.Country];
        var row = export_matrix[source] = [];
        for (var i = -1; ++i < n;) row[i] = 0;
        row[source]= d.Exports;
    });

    // Compute a unique index for each country name.
    n=0;
    totalImports=0;
    import_countries.forEach(function(d) {
        totalImports += Number(d.Imports);
        d = d.Country;
        if (!(d in i_indexByName)) {
            i_nameByIndex[n] = d;
            i_indexByName[d] = n++;
        }
    });

    import_countries.forEach(function(d) {
        var source = i_indexByName[d.Country];
        var row = import_matrix[source] = [];
        for (var i = -1; ++i < n;) row[i] = 0;
        row[source]= d.Imports;
    });

    var exportRange = angleRange*(totalExports/(totalExports + totalImports));
    var importRange = angleRange*(totalImports/(totalExports + totalImports));
    export_chord.startAngle(-(exportRange/2))
        .endAngle((exportRange/2));

    import_chord.startAngle(180-(importRange/2))
        .endAngle(180+(importRange/2));

    import_chord.matrix(import_matrix);
    export_chord.matrix(export_matrix);

    var tempLabels=[];
    var tempChords=[];

    var export_chord_groups = export_chord.groups();
    var export_chord_chords = export_chord.chords();
    export_chord_groups.sort(function(a,b) { return a.index - b.index; });
    export_chord_chords.sort(function(a,b) { return a.source.index - b.source.index; });
    for (var i = 0; i < export_chord_groups.length; i++) {
        var d = {}
        var g = export_chord_groups[i];
        var c = export_chord_chords[i];
        d.index = i;
        d.angle = (g.startAngle + g.endAngle) / 2;
        d.label = e_nameByIndex[g.index];
        d.exports = c.source.value;

        e_labels[i] = {
          angle: d.angle,
          label: d.label,
          index: i,
          exports: d.exports
        };

        e_chords[i] = {
          index: i,
          label: d.label,
          source: c.source,
          target: c.target,
          exports: d.exports
        };
    }

    var import_chord_groups = import_chord.groups();
    var import_chord_chords = import_chord.chords();
    import_chord_groups.sort(function(a,b) { return a.index - b.index; });
    import_chord_chords.sort(function(a,b) { return a.source.index - b.source.index; });
    for (var i=0; i < import_chord_groups.length; i++) {
        var d = {}
        var g = import_chord_groups[i];
        var c = import_chord_chords[i];
        d.index = i;
        d.angle = (g.startAngle + g.endAngle) / 2;
        d.label = i_nameByIndex[g.index];
        d.imports = c.source.value;
        i_labels[i] = {
          angle: d.angle,
          label: d.label,
          imports: d.imports,
          index: i
        };

        i_chords[i] = {
          index: i,
          label: d.label,
          source: c.source,
          target: c.target,
          imports: d.imports
        };
    }

    function getFirstIndex(index,indexes) {
        for (var i=0; i < topCountryCount; i++) {
            var found=false;
            for (var y=index; y < indexes.length; y++) {
                if (i==indexes[y]) {
                    found=true;
                }
            }
            if (found==false) {
                return i;
                //  break;
            }
        }
        //      console.log("no available indexes");
    }

    function getLabelIndex(name) {
        for (var i=0; i < topCountryCount; i++) {
            if (e_buffer[i].label==name) {
                return i;
                //   break;
            }
        }
        return -1;
    }


}

