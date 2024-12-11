import xml.etree.ElementTree as ET

tree = ET.parse('/home/rafael/farm_ws/src/opennav_coverage/opennav_coverage/test/cartesian_test_field.xml')
root = tree.getroot()

# Extract field information
field = root.find('Field')
area = field.get('area')
best_angle = field.get('best_angle')
# ... extract other attributes as needed

# Extract field geometry
polygon = field.find('.//gml:Polygon', namespaces={'gml': 'http://www.opengis.net/gml'})
coordinates = polygon.find('.//gml:coordinates', namespaces={'gml': 'http://www.opengis.net/gml'}).text
field_coordinates = [tuple(map(float, coord.split(','))) for coord in coordinates.split()]

# Extract rows
rows = []
for row in root.findall('Row'):
    row_id = row.get('id')
    line_string = row.find('.//gml:LineString', namespaces={'gml': 'http://www.opengis.net/gml'})
    coordinates = line_string.find('.//gml:coordinates', namespaces={'gml': 'http://www.opengis.net/gml'}).text
    row_coordinates = [tuple(map(float, coord.split(','))) for coord in coordinates.split()]
    rows.append((row_id, row_coordinates))

print('Field Coordinates:', field_coordinates)
print('Rows:', rows)