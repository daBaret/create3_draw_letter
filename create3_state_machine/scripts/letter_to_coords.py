from svgpathtools import svg2paths, Line

def svg_path_to_coordinates(svg_path):
    paths, _ = svg2paths(svg_path)
    coordinates = []

    for path in paths:
        for segment in path:
            if type(segment) == Line:
                coordinates.append(segment.start.real)
                coordinates.append(segment.start.imag)
                coordinates.append(segment.end.real)
                coordinates.append(segment.end.imag)

    return coordinates

svg_path = 's.svg'

coordinates = svg_path_to_coordinates(svg_path)

print(coordinates)

print(len(coordinates)/2)
