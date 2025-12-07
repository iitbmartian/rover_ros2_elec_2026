from matplotlib import pyplot as plt

with open("testicle_readings.txt") as f:
    brrrrrrr = bytes.fromhex(f.read())

#print(brrrrrrr)
NUM_QUADS = 5
NUM_ENCODERS = 1
NUM_ASC = 9

def f1():
    FRAME = {}
    for i in range(1, NUM_QUADS + 1):
        FRAME[f"QUAD {i} POS"] = 4
        FRAME[f"QUAD {i} DIFF 1"] = 4
        FRAME[f"QUAD {i} DIFF 2"] = 4

    FRAME["DRILL QUAD POS"] = 4
    FRAME["DRILL QUAD DIFF 1"] = 4
    FRAME["DRILL QUAD DIFF 2"] = 4

    for i in range(1, NUM_ENCODERS + 1):
        FRAME[f"MAG {i} POS"] = 2

    for i in range(1, NUM_ASC + 1):
        FRAME[f"ASC {i} VAL"] = 2

    FRAME["NEWLINE"] = 1

    return FRAME


frame = f1()

poses = []

live = True
while live:
    glop = False
    for display, bits in frame.items():
        if len(brrrrrrr) < bits:
            live = False
            break

        bop = brrrrrrr[:bits]
        brrrrrrr = brrrrrrr[bits:]
        val = int.from_bytes(bop, byteorder='big', signed=True)
        if val != 0:
            glop = True
            print(f"{display}: {val}")

        if display == "QUAD 2 DIFF 1":
            poses.append(val)
    print("-" * 67)

plt.plot(poses)
plt.show()


