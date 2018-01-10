ListView {
    anchors.fill: parent
    model: ListModel { id: model}
    delegate: Text { text: "[" + year + "]: " + population }
    Component.onCompleted: {
        var xhr = new XMLHttpRequest;
        xhr.open("GET",
                 "http://inqstatsapi.inqubu.com/?api_key=YOURKEYHERE&data=population&countries=us");
        xhr.onreadystatechange = function() {
            if (xhr.readyState === XMLHttpRequest.DONE) {
                var data = JSON.parse(xhr.responseText);
                model.clear();
                var list = data[0]["population"];
                for (var i in list) {
                  model.append({year: list[i]["year"],
                                population: list[i]["data"]});
                }
            }
        }
        xhr.send();
    }
}
