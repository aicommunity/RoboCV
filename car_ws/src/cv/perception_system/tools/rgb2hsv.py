import colorsys

(r, g, b) = (0, 0, 142)

def rgb2hsv(ri, gi, bi):
    # Normalize
    (r, g, b) = (ri / 255, gi / 255, bi / 255)

    # Convert to hsv
    (h, s, v) = colorsys.rgb_to_hsv(r, g, b)

    # Expand HSV range
    return (int(h * 179), int(s * 255), int(v * 255))

(h, s, v) = rgb2hsv(r, g, b)
print(h+1, s, v)