
function buildChords(y,m) {

    countries=countriesGrouped[y].values[m].values;

    countries.sort(function (a,b) {
        //Descending Sort
        if (a.Exports > b.Exports) return -1;
        else if (a.Exports < b.Exports) return 1;
        else return 0;
    });

    export_countries=countries.slice(0,topCountryCount);

    countries.sort(function (a,b) {
        //Descending Sort
        if (a.Imports > b.Imports) return -1;
        else if (a.Imports < b.Imports) return 1;
        else return 0;
    });

    import_countries=countries.slice(0,topCountryCount);

    var  import_matrix = [],
        export_matrix = [];

    e_buf_indexByName=e_indexByName;
    i_buf_indexByName=i_indexByName;

    e_indexByName=[];
    e_nameByIndex=[];
    i_indexByName=[];
    i_nameByIndex=[];
    n = 0;

    // Compute a unique index for each package name
    totalExports=0;
    export_countries.forEach(function(d) {
        totalExports+= Number(d.Exports);
        d = d.Country;
        if (!(d in e_indexByName)) {
            e_nameByIndex[n] = d;
            e_indexByName[d] = n++;
        }
    });

    export_countries.forEach(function(d) {
        var source = e_indexByName[d.Country],
            row = export_matrix[source];
        if (!row) {
            row = export_matrix[source] = [];
            for (var i = -1; ++i < n;) row[i] = 0;
        }
        row[e_indexByName[d.Country]]= d.Exports;
    });

    // Compute a unique index for each country name.
    n=0;
    totalImports=0;
    import_countries.forEach(function(d) {
        totalImports+= Number(d.Imports);
        d = d.Country;
        if (!(d in i_indexByName)) {
            i_nameByIndex[n] = d;
            i_indexByName[d] = n++;
        }
    });

    import_countries.forEach(function(d) {
        var source = i_indexByName[d.Country],
            row = import_matrix[source];
        if (!row) {
            row = import_matrix[source] = [];
            for (var i = -1; ++i < n;) row[i] = 0;
        }
        row[i_indexByName[d.Country]]= d.Imports;
    });

    var exportRange=angleRange*(totalExports/(totalExports + totalImports));
    var importRange=angleRange*(totalImports/(totalExports + totalImports));
    export_chord.startAngle(-(exportRange/2))
        .endAngle((exportRange/2));

    import_chord.startAngle(180-(importRange/2))
        .endAngle(180+(importRange/2));

    import_chord.matrix(import_matrix);
    export_chord.matrix(export_matrix);

    var ec_groups = export_chord.groups();
    var ec_chords = export_chord.chords();
    ec_groups.sort(function(a,b) { return a.index - b.index; });
    ec_chords.sort(function(a,b) { return a.source.index - b.source.index; });
    for (var i=0; i < ec_groups.length; i++) {
        var d={}
        var g=ec_groups[i];
        var c=ec_chords[i];
        d.index=i;
        d.angle= (g.startAngle + g.endAngle) / 2;
        d.label = e_nameByIndex[g.index];
        d.exports= c.source.value;

        // create a new object instead of overwriting
        // overwriting changes the data bound to d3 objects, which is not what
        // we want
        e_labels[i] = {};
        e_labels[i].angle = d.angle;
        e_labels[i].label = d.label;
        e_labels[i].index = i;
        e_labels[i].exports = d.exports;

        e_chords[i] = {};
        e_chords[i].index = i;
        e_chords[i].label = d.label;
        e_chords[i].source = c.source;
        e_chords[i].target = c.target;
        e_chords[i].exports = d.exports;
    }

    var ic_groups = import_chord.groups();
    var ic_chords = import_chord.chords();
    ic_groups.sort(function(a,b) { return a.index - b.index; });
    ic_chords.sort(function(a,b) { return a.source.index - b.source.index; });
    for (var i=0; i < ic_groups.length; i++) {
        var d={}
        var g=import_chord.groups()[i];
        var c=import_chord.chords()[i];
        d.index=i;
        d.angle= (g.startAngle + g.endAngle) / 2;
        d.label = i_nameByIndex[g.index];
        d.imports = c.source.value;

        // create a new object instead of overwriting
        // overwriting changes the data bound to d3 objects, which is not what
        // we want
        i_labels[i] = {};
        i_labels[i].angle = d.angle;
        i_labels[i].label = d.label;
        i_labels[i].imports = d.imports;
        i_labels[i].index = i;

        i_chords[i] = {};
        i_chords[i].index = i;
        i_chords[i].label = d.label;
        i_chords[i].source = c.source;
        i_chords[i].target = c.target;
        i_chords[i].imports = d.imports;
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

