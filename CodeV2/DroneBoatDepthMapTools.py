def color(cType, val):
    if cType == "red":
        val += 0
    if cType == "green":
        val += float(2) / 3
    if cType == "blue":
        val += float(1) / 3

    if val >= 1:
        val -= 1

    if val <= float(1) / 3:
        return 255 * (float(1) / 3 - val) * 3
    if val >= float(2) / 3:
        return 255 * (val - float(2) / 3) * 3

    return 0

def colorificate(dmin, dval, dmax):
    d = (dval - dmin) / (dmax - dmin)
    red = round(color("red", d))
    green = round(color("green", d))
    blue = round(color("blue", d))

    opacity = 50

    return red, green, blue, opacity


def creatediv(x, y, xl, yl, c):
    start = '<div id="MapPixel" style="'
    end = '"></div>'
    px = 'px; '

    return (start +
            'top: ' + str(y) + px +
            'left: ' + str(x) + px +
            'width: ' + str(xl) + px +
            'height: ' + str(yl) + px +
            'background: rgba' + str(c) + '; ' +
            'position: absolute' +
            end)

class DepthMap:
    def __init__(self, mapinfo, droneBoatGPS):
        self.depthdata = []
        self.depthjson = []
        self.lastPoint = None

        self.MapArray = []
        self.PopulatedMap = []
        self.mapinfo = mapinfo
        self.droneBoatGPS = droneBoatGPS #for fprint

    def GetPoint(self, x, y):
        try:
            return self.MapArray[y][x]
        except (Exception,):
            return None #point out of range

    def GetAvgTile(self, x, y, size):
        total = 0
        count = 0

        for ypos in range((y - size), (y + size)):
            for xpos in range((x - size), (x + size)):
                point = self.GetPoint(xpos, ypos)
                if point is not None:
                    total += point
                    count += 1
        if count == 0:
            return None
        return round((float(total) / count), 6)

    def PopulateTile(self, x, y):
        for size in range(0, 3): #focus on middle tiles
            t = self.GetAvgTile(x, y, size)
            if t is not None:
                self.PopulatedMap[y][x] = t
                return

    def makeJS(self):
        start = "<script>window.onload=OnLoad;\n"
        end = "</script>"
        func = (
            "function OnLoad() \n { \n //alert(navigator.geolocation); \n if(navigator.geolocation) \n { \n //alert('working!'); \n navigator.geolocation.getCurrentPosition(move);\n } \n else \n { \n //alert('NO GPS') \n } \n}\n  function move(pos) \n {\n //alert('HELLO!');\n moveMark(pos.coords.latitude, pos.coords.longitude);\n }\n")
        func2 = (
                "function moveMark(gpsN, gpsW)\n { var N = (((gpsN-gpsBX)/(gpsTX-gpsBX))*mapH) - 20; \n var W = (((gpsW-gpsBY)/(gpsTY-gpsBY))*mapW) - 10; \n  N = N.toFixed(0); \n  W = W.toFixed(0); \n var elem = document.getElementById(" + '"phone"' + "); \n //alert(elem); \n //alert(N); \n elem.style.top = N + 'px'; \n  elem.style.left = W + 'px';\n elem.style.position = 'absolute';\n }\n")
        html = '<img id="phone" src="markerPhone.png" height=20 width=20>'
        vardef = ('var mapW = ' + str(self.mapinfo["MapW"]) + ';\n' +
                  'var mapH = ' + str(self.mapinfo["MapH"]) + ';\n' +
                  'var gpsBX = ' + str(self.mapinfo["TopN"]) + ';\n' +
                  'var gpsBY = ' + str(self.mapinfo["TopW"]) + ';\n' +
                  'var gpsTX = ' + str(self.mapinfo["BottomN"]) + ';\n' +
                  'var gpsTY = ' + str(self.mapinfo["BottomW"]) + ';\n')
        return html + start + vardef + func + func2 + end

    def PlaceOnMap(self, X, Y, Depth):
        truex = int(round(((X - self.mapinfo["TopW"]) / (self.mapinfo["BottomW"] - self.mapinfo["TopW"])) * self.mapinfo["MapW"]))
        truey = int(round(((Y - self.mapinfo["TopN"]) / (self.mapinfo["BottomN"] - self.mapinfo["TopN"])) * self.mapinfo["MapH"]))

        try:
            self.MapArray[truey][truex] = Depth
        except Exception as e:
            self.droneBoatGPS.fprint("Can't place value on map! Error in Loc y/Map y, Loc x, Map x")
            self.droneBoatGPS.fprint(e)
            self.droneBoatGPS.fprint((str(truey) + " " + str(len(self.MapArray))))
            self.droneBoatGPS.fprint((str(truex) + " " + str(len(self.MapArray[0]))))

    def MakeDepthMap(self):
        shrink = float(1) / self.mapinfo["Res"]
        scale = self.mapinfo["Res"]

        self.MapArray = []
        self.PopulatedMap = []

        for x in range(0, int(round(self.mapinfo["MapH"] * scale))):
            tempmap = []
            for y in range(0, int(round(self.mapinfo["MapW"] * scale))):
                tempmap.append(None)
            self.MapArray.append(tempmap)

        for x in range(0, int(round(self.mapinfo["MapH"] * scale))):
            tempmap = []
            for y in range(0, int(round(self.mapinfo["MapW"]))):
                tempmap.append(None)
            self.PopulatedMap.append(tempmap)

        for line in self.depthjson:
            if line["DPT"] is not None:
                self.PlaceOnMap(float(line["GPSW"]), float(line["GPSN"]), float(line["DPT"]))

        for y in range(0, len(self.MapArray)):
            for x in range(0, len(self.MapArray[y])):
                self.PopulateTile(x, y)

        start = '<!DOCTYPE html><html><body><img src='
        more = ' height="'
        evenmore = '" width="'
        stillmore = '">'
        end = "</body></html>"
        divlist = []
        for y in range(0, len(self.PopulatedMap)):
            for x in range(0, len(self.PopulatedMap[y])):
                d = self.PopulatedMap[y][x]
                if d is not None:
                    try:
                        divlist.append(creatediv((x * shrink), (y * shrink), shrink, shrink,
                                                 colorificate(0, d, self.mapinfo["MaxDepth"])))
                    except (Exception, ) as e:
                        print(e)  # fail quietly
        fullstart = start + self.mapinfo["Source"] + more + str(self.mapinfo["MapH"]) + evenmore + str(self.mapinfo["MapW"]) + stillmore
        full = fullstart
        for div in divlist:
            full = full + div
        full = full + self.makeJS()
        full = full + end
        return full