function update(y,m) {

    updateMonths(y,m);

    buildChords(y,m);

   // mainLabel.style("font-size",innerRadius *.05);

    eText = eGroup.selectAll("g.group")
        .data(e_labels, function (d) {
            return d.label;
        });

    iText = iGroup.selectAll("g.group")
        .data(i_labels, function (d) {
            return d.label;
        });

    eChords=eGroup.selectAll("g.chord")
        .data(e_chords, function (d) {
            return d.label;
        });

    iChords=iGroup.selectAll("g.chord")
        .data(i_chords, function (d) {
            return d.label;
        });

    var td=((monthlyExports[y*12+m] - monthlyImports[y*12+m]))*-1
    var fs=innerRadius *.1;
    td=formatCurrency(td);

    dGroup.transition()
        .select("text")
        .delay(delay)
        .text(td)
        .attr("transform", "translate(" + (outerRadius - (td.length * fs/2)/2) + ","  + (outerRadius*1.1) +")")
        .style("font-size", fs + "px");

    eText.enter()
        .append("g")
        .attr("class", "group")
        .append("text")
        .attr("class","export")
        .attr("dy", ".35em")
        .attr("text-anchor", function(d) { return d.angle > Math.PI ? "end" : null; })
        .attr("transform", function(d) {
            return "rotate(" + (d.angle * 180 / Math.PI - 90) + ")"
                + "translate(" + (innerRadius + 6) + ")"
                + (d.angle > Math.PI ? "rotate(180)" : "");
        })
        .text(function(d) { return  (d.index+1)  + ". " + d.label; })
        .on("mouseover", function (d) { node_onMouseOver(d); })
        .on("mouseout", function (d) {node_onMouseOut(d); });

    eText.transition()
        .duration(delay-10)
        .select("text")
        .attr("dy", ".35em")
        .attr("text-anchor", function(d) { return d.angle > Math.PI ? "end" : null; })
        .attr("transform", function(d) {
            return "rotate(" + (d.angle * 180 / Math.PI - 90) + ")"
                + "translate(" + (innerRadius + 6) + ")"
                + (d.angle > Math.PI ? "rotate(180)" : "");
        })
        .text(function(d) { return  (d.index+1)  + ". " + d.label; });

    eText.exit().remove();

    eChords.enter()
        .append("g")
        .attr("class","chord")
        .append("path")
        .attr("class","chord")
        .style("stroke", function(d) { return d3.rgb(getExportColor(d.source.index)).darker(); })
        .style("fill", function(d) { return getExportColor(d.source.index); })
        .style("fill-opacity", function (d,i) { return .85*(topCountryCount- d.index)/topCountryCount})
        .attr("d", d3.svg.arc_chord().radius(innerRadius))
        .style("opacity",0)
        .on("mouseover", function (d) { node_onMouseOver(d); })
        .on("mouseout", function (d) {node_onMouseOut(d); });


    eChords.transition()
        .select("path")
        .duration(delay)
        .attr("d", d3.svg.arc_chord().radius(innerRadius))
        .style("stroke", function(d) { return d3.rgb(getExportColor(d.source.index)).darker(); })
        .style("fill", function(d) { return getExportColor(d.source.index); })
        .style("stroke-opacity", function (d,i) { return Math.max(.85*(topCountryCount-d.index)/topCountryCount,.2);})
        .style("fill-opacity", function (d,i) { return .85*(topCountryCount-d.index)/topCountryCount})
        .style("opacity",1);


    eChords.exit()
        .remove();

    iText.enter()
        .append("g")
        .attr("class", "group")
        .append("text")
        .attr("class","import")
        .attr("dy", ".35em")
        .attr("text-anchor", function(d) { return d.angle > Math.PI ? "end" : null; })
        .attr("transform", function(d) {
            return "rotate(" + (d.angle * 180 / Math.PI - 90) + ")"
                + "translate(" + (innerRadius + 6) + ")"
                + (d.angle > Math.PI ? "rotate(180)" : "");
        })
        .text(function(d) { return  (d.index+1)  + ". " + d.label; })
        .on("mouseover", function (d) { node_onMouseOver(d); })
        .on("mouseout", function (d) {node_onMouseOut(d); });

    iText.transition()
        .select("text")
        .duration(delay-10)
        .attr("dy", ".35em")
        .attr("text-anchor", function(d) { return d.angle > Math.PI ? "end" : null; })
        .attr("transform", function(d) {
            return "rotate(" + (d.angle * 180 / Math.PI - 90) + ")"
                + "translate(" + (innerRadius + 6) + ")"
                + (d.angle > Math.PI ? "rotate(180)" : "");
        })
        .text(function(d) { return  (d.index+1)  + ". " + d.label; });

    iText.exit()
        .attr("class", "exit")
        .transition()
        .duration(delay)
        .attr("y",0)
        .attr("fill-opacity",1e-6)
        .remove();

    iChords.enter()
        .append("g")
        .attr("class","chord")
        .append("path")
        .attr("class","chord")
        .style("stroke", function(d) { return d3.rgb(getImportColor(d.source.index)).darker(); })
        .style("stroke-opacity", function (d,i) { return Math.max(.85*(topCountryCount-d.index)/topCountryCount,.2);})
        .style("fill", function(d) { return getImportColor(d.source.index); })
        .style("fill-opacity", function (d,i) { return .7*(topCountryCount- d.index)/topCountryCount})
        .attr("d", d3.svg.arc_chord().radius(innerRadius))
        .on("mouseover", function (d) { node_onMouseOver(d); })
        .on("mouseout", function (d) {node_onMouseOut(d); });

    iChords.transition()
        .select("path")
        .duration(delay-10)
        .attr("d", d3.svg.arc_chord().radius(innerRadius))
        .style("stroke", function(d) { return d3.rgb(getImportColor(d.source.index)).darker(); })
        .style("fill", function(d) { return  getImportColor(d.source.index); })
        .style("stroke-opacity", function (d,i) { return Math.max(.85*(topCountryCount-d.index)/topCountryCount,.2);})
        .style("fill-opacity", function (d,i) { return .7*(topCountryCount- d.index)/topCountryCount});


    iChords.exit()
        .remove();

}

function updateMonths(y,m) {

    var monthAxis=mGroup.selectAll("g.month")
        .data(months);

    monthEnter= monthAxis.enter()
        .append("g")
        .attr("class","month");

    monthEnter.append("line")
        .attr("x1",function (d,i) {
            return i*monthOffset;
        })
        .attr("x2",function (d,i) { return i*monthOffset; })
        .attr("y1",function (d,i) {
            var ratio=(y*12+m)-i;
            if (ratio < 0) ratio=ratio*-1;
            if (ratio==0)
                return 0;
            else if (ratio==1)
                return 4;
            else if (ratio==2)
                return 8;
            else if (ratio==3)
                return 11;
            else if (ratio==4)
                return 14;
            else if (ratio==5)
                return 15;
            else if (ratio==6)
                return 15;
            else
                return 16;

        })
        .attr("y2",22)
        .attr("shape-rendering","crispEdges")
        .style("stroke-opacity", function (d,i) {
            var ratio=(y*12+m)-i;
            if (ratio < 0) ratio=ratio*-1;
            if (ratio==0)
                return 1;
            else if (ratio==1)
                return .9;
            else if (ratio==2)
                return .8;
            else if (ratio==3)
                return .7;
            else if (ratio==4)
                return .6;
            else if (ratio==5)
                return .5;
            else if (ratio==6)
                return .4;
            else
                return .3;

        })
        .style("stroke","#000");



    monthEnter.append("text")
        .attr("transform",function (d,i) { return "translate (" + String(i*monthOffset-10) + ",-2)"; })
        .text(function(d,i) { return monthsMap[i % 12]; })
        .style("fill-opacity",function (d,i) { return (i==0) ? 1:0;});

    monthEnter.append("text")
        .attr("transform",function (d,i) { return "translate (" + String(i*monthOffset-10) + ",33)"; })
        .text(function(d,i) {
            if ((i==0) || (i % 12==0)) {
                return String(baseYear + Math.floor(i/12));
            }
            else
                return "";
        })
        .on("click",function (d) {
            year= Math.floor(d.index/12);
            month=0;
            if (running==true) stopStart();
            update(year,month);
            //          console.log("y=" + y + " m=" + m);
        });

    monthUpdate=monthAxis.transition();

    monthUpdate.select("text")
        .delay(delay/2)
        .style("fill-opacity",function (d) {
            if (d.index==(y*12+m)) {
                return 1;
            }
            else
                return 0;
        });

    monthUpdate.select("line")
        .delay(delay/2)
        .attr("y1",function (d,i) {
            var ratio=(y*12+m)-i;
            if (ratio < 0) ratio=ratio*-1;
            if (ratio==0)
                return 0;
            else if (ratio==1)
                return 4;
            else if (ratio==2)
                return 8;
            else if (ratio==3)
                return 11;
            else if (ratio==4)
                return 14;
            else if (ratio==5)
                return 15;
            else if (ratio==6)
                return 15;
            else
                return 16;

        })
        .style("stroke-width",function (d,i) {
            var ratio=(y*12+m)-i;
            if (ratio < 0) ratio=ratio*-1;
            if (ratio==0)
                return 1.5;
            else
                return 1;
        })
        .style("stroke-opacity", function (d,i) {
            var ratio=(y*12+m)-i;
            if (ratio < 0) ratio=ratio*-1;
            if (ratio==0)
                return 1;
            else if (ratio==1)
                return .9;
            else if (ratio==2)
                return .8;
            else if (ratio==3)
                return .7;
            else if (ratio==4)
                return .6;
            else if (ratio==5)
                return .5;
            else if (ratio==6)
                return .4;
            else
                return .3;

        })
        .style("stroke","#000");

}


function getExportColor(i) {
    var country=e_nameByIndex[i];
    if (e_colorByName[country]==undefined) {
        e_colorByName[country]=e_fill(i);
    }

    return e_colorByName[country];
}

function getImportColor(i) {
    var country=i_nameByIndex[i];
    if (i_colorByName[country]==undefined) {
        i_colorByName[country]=i_fill(i);
    }

    return i_colorByName[country];
}
