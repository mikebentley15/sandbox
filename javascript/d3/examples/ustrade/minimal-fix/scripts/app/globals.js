
var maxWidth=Math.max(600,Math.min(window.innerWidth,window.innerHeight)-50);

var outerRadius = (maxWidth / 2),
    innerRadius = outerRadius - 100,
    monthWidth=Math.max(400,(innerRadius*2)-250);


var iText,iChords,eText,eChords;

var angleRange=320,
    baseYear=2001,
    maxMonth=1,
    maxYear=12,
    monthOffset=(monthWidth)/(maxYear*12+maxMonth),
    countries,
    e_labels=[],
    e_chords=[],
    i_labels=[],
    i_chords=[],
    topCountryCount=20,
    e_buf_indexByName={},
    e_indexByName = {},
    e_nameByIndex = {},
    i_indexByName = {},
    i_nameByIndex = {},
    i_buf_indexByName={},
    export_countries=[],
    import_countries=[],
    e_colorByName={},
    i_colorByName={},
    months=[],
    monthlyExports=[],
    monthlyImports=[],
    countriesGrouped,
    delay=1200,
    refreshIntervalId,
    year= 0,
    month=-1,
    running=true,
    formatNumber = d3.format(",.0f"),
    formatCurrency = function(d) { return "-$" + formatNumber(d)},
    eTextUpdate,
    eChordUpdate,
    TextUpdate,
    iChordUpdate;

var toolTip = d3.select(document.getElementById("toolTip"));
var header = d3.select(document.getElementById("head"));
var header1 = d3.select(document.getElementById("header1"));
var header2 = d3.select(document.getElementById("header2"));

var e_fill= d3.scale.ordinal().range(["#00AC6B","#20815D","#007046","#35D699","#60D6A9"]);
var i_fill= d3.scale.ordinal().range(["#EF002A","#B32D45","#9B001C","#F73E5F","#F76F87"]);

var monthsMap=["Jan","Feb","Mar","Apr","May","Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];

d3.select(document.getElementById("bpg"))
    .style("min-width",(outerRadius*2 + 150) + "px");


var playPause=d3.select(document.getElementById("playPause"));

d3.select(document.getElementById("imgDiv"))
    .style("left",((outerRadius-monthWidth/2))+"px");

var svg = d3.select(document.getElementById("svgDiv"))
    .style("width", (outerRadius*2) + "px")
    .style("height", (outerRadius*2+200) + "px")
    .append("svg")
    .attr("id","svg")
    .style("width", (outerRadius*2) + "px")
    .style("height", (outerRadius*2+200) + "px");


var export_chord = d3.layout.arc_chord()
    .padding(.05)
    .sortSubgroups(d3.descending)
    .sortChords(d3.descending)
    .yOffsetFactor(-0.8);

var import_chord = d3.layout.arc_chord()
    .padding(.05)
    .yOffsetFactor(0.7)
    .sortSubgroups(d3.descending)
    .sortChords(d3.descending);

var arc = d3.svg.arc()
    .innerRadius(innerRadius)
    .outerRadius(innerRadius + 5);


var dGroup = svg.append("g")
    .attr("class","mainLabel")

dGroup.append("text")
    .attr("class","mainLabel")
    .attr("transform", "translate(" + (outerRadius - 20) + ","  + (outerRadius + 30) +")")
    .style("font-size","0px");

dGroup.append("text")
    .attr("class","secondLabel")
    .attr("transform", "translate(" + (outerRadius - 90) + ","  + (outerRadius * 1.15) +")")
    .text("* Cumulative trade imbalance since 2001")
    .style("font-size","0px");

var gY=(outerRadius-(innerRadius *.8/2));

gradientGroup =svg.append("g")
    .attr("class","gradient")
    .attr("transform","translate(" + (outerRadius-6) + "," + (gY+70)  + ")" );

gradientGroup.append("rect")
    .attr("height",((outerRadius + innerRadius *.7/2)-gY))
    .attr("width",0)
    .style("fill","url(#gradient1)");

var mGroup=svg.append("g")
    .attr("class","months")
    .style("cursor","pointer")
    .attr("transform", "translate(" + (outerRadius-monthWidth/2-20) + ","  + 20 + ")");

var eGroup=svg.append("g")
    .attr("class","exports")
    .attr("transform", "translate(" + outerRadius + "," + (outerRadius+70) + ")");

var iGroup=svg.append("g")
    .attr("class","imports")
    .attr("transform", "translate(" + outerRadius + "," + (outerRadius+70) + ")");

