function createVerticalGradient(svgId,gradientId,stops) {
        var svg=document.getElementById(svgId);
        var svgNS = svg.namespaceURI;
        var grad  = document.createElementNS(svgNS,'linearGradient');
        grad.setAttribute('x1','0%');
        grad.setAttribute('x2','0%');
        grad.setAttribute('y1','0%');
        grad.setAttribute('y2','100%');
        grad.setAttribute('id',gradientId);
        for (var i=0;i<stops.length;i++){
            var attrs = stops[i];
            var stop = document.createElementNS(svgNS,'stop');
            for (var attr in attrs){
                if (attrs.hasOwnProperty(attr)) stop.setAttribute(attr,attrs[attr]);
            }
            grad.appendChild(stop);
        }

        var defs = document.querySelector('defs') || svg.insertBefore( document.createElementNS(svgNS,'defs'), svg.firstChild );
        return defs.appendChild(grad);

    }
