import numpy as np
from PIL import Image

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def pol2cart_center(rho, phi, number_of_leds):
    x = number_of_leds + rho * np.cos(phi)
    y = number_of_leds + rho * np.sin(phi)
    return(x, y)




if __name__ == "__main__":
    number_of_leds = 20
    number_of_steps = 120
    step_size = 2*np.pi/number_of_steps #30 degrees

    output_pos = []
    output_color = []

    f = open('output_bits_black_120_frames.txt', 'w')


    # PIL accesses images in Cartesian co-ordinates, so it is Image[columns, rows]
    img = Image.open("./DSO_small_black.jpg")
    img_out = Image.new( 'RGB', (40,40), "black") # create a new black image
    pixels = img.load() # create the pixel map
    pixels_out = img_out.load()
    for i in range(number_of_steps):
        f.write("int frame%d[20][4] = {" %i)
        angle = i * step_size
        for j in range(number_of_leds):
            #angle = (i/number_of_steps) * 2 *np.pi

            pos = pol2cart_center(j,angle,number_of_leds)
            #pos = pol2cart_center(j,i,number_of_leds)
            output_pos.append(pos)
            output_color.append(pixels[pos])
            f.write("{%d, %d, %d" % (pixels[pos][0], pixels[pos][1], pixels[pos][2]))
            if(j != number_of_leds-1):
                f.write(", 31},\n")
            else:
                f.write(", 31}\n")
            #print(output_color)
            pixels_out[pos] = pixels[pos]
        f.write("};\n")
    #print(output_color)
    f.close()
    img_out.save('output_black_dso_120_40x40.jpg','JPEG')
    img_out.show()
