import fileinput

x,y,z = [],[],[]

length = 100

for line in fileinput.input():
    if line.startswith('x'):
        x.append(float(line.split("x: ")[1]))
        if len(x) > length:
            x.pop(0)
        print(f"x {x[-1]:.4f}, {sum(x)/len(x):.4f}")

    elif line.startswith('y'):
        y.append(float(line.split("y: ")[1]))
        if len(y) > length:
            y.pop(0)
        print(f"x {y[-1]:.4f}, {sum(y)/len(y):.4f}")

    elif line.startswith('z'):
        z.append(float(line.split("z: ")[1]))
        if len(z) > length:
            z.pop(0)
        print(f"x {z[-1]:.4f}, {sum(z)/len(z):.4f}")
    pass
