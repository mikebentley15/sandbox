
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

    var tempLabels=[];
    var tempChords=[];

    for (var i=0; i < e_labels.length; i++) {
        e_labels[i].label='null';
        e_chords[i].label='null';
    }

    for (var i=0; i < export_chord.groups().length; i++) {
        var d={}
        var g=export_chord.groups()[i];
        var c=export_chord.chords()[i];
        d.index=i;
        d.angle= (g.startAngle + g.endAngle) / 2;
        d.label = e_nameByIndex[g.index];
        d.exports= c.source.value;
        var bIndex=e_buf_indexByName[d.label];
        if (typeof bIndex != 'undefined') {  //Country already exists so re-purpose node.
            e_labels[bIndex].angle= d.angle;
            e_labels[bIndex].label= d.label;
            e_labels[bIndex].index= i;
            e_labels[bIndex].exports= d.exports;

            e_chords[bIndex].index= i;
            e_chords[bIndex].label= d.label;
            e_chords[bIndex].source= c.source;
            e_chords[bIndex].target= c.target;
            e_chords[bIndex].exports = d.exports;

        }
        else { //Country doesnt currently exist so save for later
            tempLabels.push(d);
            tempChords.push(c);
        }
    }

    //Now use up unused indexes
    for (var i=0; i < e_labels.length; i++) {
        if (e_labels[i].label=="null") {
            var o=tempLabels.pop();
            e_labels[i].index=e_indexByName[o.label];
            e_labels[i].label= o.label;
            e_labels[i].angle= o.angle;
            e_labels[i].exports= o.exports;

            var c=tempChords.pop();
            e_chords[i].label= o.label;
            e_chords[i].index= i;
            e_chords[i].source= c.source;
            e_chords[i].target= c.target;
            e_chords[i].exports= c.exports;

        }
    }


    tempLabels=[];
    tempChords=[];

    for (var i=0; i < i_labels.length; i++) {
        i_labels[i].label='null';
        i_chords[i].label='null';
    }

    for (var i=0; i < import_chord.groups().length; i++) {
        var d={}
        var g=import_chord.groups()[i];
        var c=import_chord.chords()[i];
        d.index=i;
        d.angle= (g.startAngle + g.endAngle) / 2;
        d.label = i_nameByIndex[g.index];
        d.imports = c.source.value;
        var bIndex=i_buf_indexByName[d.label];
        if (typeof bIndex != 'undefined') {  //Country already exists so re-purpose node.
            i_labels[bIndex].angle= d.angle;
            i_labels[bIndex].label= d.label;
            i_labels[bIndex].imports= d.imports;
            i_labels[bIndex].index= i;

            i_chords[bIndex].index= i;
            i_chords[bIndex].label= d.label;
            i_chords[bIndex].source= c.source;
            i_chords[bIndex].target= c.target;
            i_chords[bIndex].imports= d.imports;

        }
        else { //Country doesnt currently exist so save for later
            tempLabels.push(d);
            tempChords.push(c);
        }
    }

    //Now use up unused indexes
    for (var i=0; i < i_labels.length; i++) {
        if (i_labels[i].label=="null") {
            var o=tempLabels.pop();
            i_labels[i].index=i_indexByName[o.label];
            i_labels[i].label= o.label;
            i_labels[i].angle= o.angle;
            i_labels[i].imports= o.imports;

            var c=tempChords.pop();
            i_chords[i].label= o.label;
            i_chords[i].index= i;
            i_chords[i].source= c.source;
            i_chords[i].target= c.target;
            i_chords[i].imports= c.imports;

        }
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

