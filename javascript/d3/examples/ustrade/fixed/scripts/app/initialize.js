function initialize() {

    var count=maxYear*12+maxMonth;

    for (var i=0; i < count; i++) {
        var o={};
        o.index=i;
        o.month=monthsMap[i % 12];
        o.year=baseYear + Math.floor(i/12);
        months.push(o);
    }

    for (var i=0; i < topCountryCount; i++) {
        var l={};
        l.index=i;
        l.label="null";
        l.angle=0;
        e_labels.push(l);

        var c={}
        c.label="null";
        c.source={};
        c.target={};
        e_chords.push(c);

        var l1={};
        l1.index=i;
        l1.label="null";
        l1.angle=0;
        i_labels.push(l1);

        var c1={}
        c1.label="null";
        c1.source={};
        c1.target={};
        i_chords.push(c1);
    }

    createVerticalGradient('svg','gradient1',[
        {offset:'0%', 'stop-color':'#00AC6B'},
        {offset: '40%', 'stop-color':'#FFFFFF', 'stop-opacity':'0' },
        {offset: '60%', 'stop-color':'#FFFFFF', 'stop-opacity':'0' },
        {offset:'100%','stop-color':'#9B001C'}]);



    gradientGroup.transition()
        .select("rect")
        .delay(delay*1.5)
        .attr("width",12);

    dGroup.transition()
        .selectAll("text")
        .delay(delay*1.5)
        .style("font-size","10px");
}
