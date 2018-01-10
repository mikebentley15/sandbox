playPause.on("click",stopStart);

function stopStart() {
    if (running==true) {
        running=false;
        clearInterval(refreshIntervalId);
        playPause.attr("src","images/play_bw.png");
        eChords.interrupt();
        iChords.interrupt();
        iText.interrupt();
        eText.interrupt();

    }
    else {
        running=true;
        playPause.attr("src","images/pause_bw.png");
        update(year,month);
        refreshIntervalId=setInterval(run,delay);
    }
}

function node_onMouseOver(d) {
    var t;
    if (typeof d.imports == "undefined") {
        t="Exports: " + formatCurrency(Number(d.exports)*1000000);
    }
    else {
        t="Imports: " + formatCurrency(Number(d.imports)*1000000);
    }
    toolTip.transition()
        .duration(200)
        .style("opacity", ".9");
    header.text((d.index+1) + ". " + d.label);
    header1.text(monthsMap[month] + " " + (baseYear+year));
    header2.text(t);
    toolTip.style("left", (d3.event.pageX+15) + "px")
        .style("top", (d3.event.pageY-75) + "px");
}

function node_onMouseOut(d) {

    toolTip.transition()									// declare the transition properties to fade-out the div
        .duration(500)									// it shall take 500ms
        .style("opacity", "0");							// and go all the way to an opacity of nil

}
/** Returns an event handler for fading a given chord group. */
function fade(opacity) {

    return;

    return function(g, i) {
        svg.selectAll("path.chord")
            .filter(function(d) {
                //  return true;
                return d.source.index != i && d.target.index != i;
            })
            .transition()
            .style("opacity", opacity);
    };
}
