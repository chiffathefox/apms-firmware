
from matplotlib import pyplot
from argparse import ArgumentParser


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("file", type=str)
    args = parser.parse_args()

    heap = []
    stack = []

    for line in open(args.file, encoding="latin-1"):
        magic = "meminfo:"

        if magic not in line:
            continue

        try:
            line = line.rstrip()
            p = line.split(magic)
            task = p[0].split(": ")[-2].split(" ")[-1]
            d = p[1].split(": ")[1].split(", ")
            h = int(d[0].split(" ")[-1])
            s = int(d[1].split(" ")[-1])

            heap.append(h)
            stack.append(s)
        except ValueError:
            pass

    x = list(range(len(heap)))
    _, ax = pyplot.subplots()
    ax.xaxis.grid(linestyle="dashed")
    ax.yaxis.grid(linestyle="dashed")
    ax.plot(x, heap, label="heap")
    ax.plot(x, stack, label="stack")
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand",
                borderaxespad=0.)

    pyplot.show()
