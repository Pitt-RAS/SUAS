
class point:
	def __init__(self,lat,lon):
		self.lat = lat
		self.lon = lon

def minPoint(myList = [], *args):
	min = point(myList[0].lat, myList[0].lon)
	for x in myList:
		if min.lat > x.lat:
			min.lat = x.lat
		if min.lon > x.lon:
			min.lon = x.lon
	return min

def translate(min, myList = [], *args):
	for x in myList:
		x.lat = (x.lat - min.lat)*10000
		x.lon = (x.lon - min.lon)*10000
		
def printPoints(myList = [], *args):
	for x in myList:
		print "Lat: ", x.lat, " Lon: ",x.lon
		
def main():
	list = []
	list.append( point(38.1462694444444,-76.4281638888889))
	list.append( point(38.151625,-76.4286833333333))
	list.append( point(38.1518888888889,-76.4314666666667))
	list.append( point(38.1505944444444,-76.4353611111111))
	list.append( point(38.1475666666667,-76.4323416666667))
	list.append( point(38.1446666666667,-76.4329472222222))
	list.append( point(38.1432555555556,-76.4347666666667))
	list.append( point(38.1404638888889,-76.4326361111111))
	list.append( point(38.1407194444444,-76.4260138888889))
	list.append( point(38.1437611111111,-76.4212055555556))
	list.append( point(38.1473472222222,-76.4232111111111))
	list.append( point(38.1461305555556,-76.4266527777778))
	
	min = minPoint(list)
	
	translate(min,list)
	
	printPoints(list)
	
main()
