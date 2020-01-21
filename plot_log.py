
from matplotlib import pyplot
from matplotlib import ticker
from argparse import ArgumentParser
import numpy


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("file", type=str)
    args = parser.parse_args()

    data = {}
    units = {}

    for line in open(args.file, encoding="latin-1"):
        magic = "main: data: "

        if magic not in line:
            continue

        line = line.split(magic)[1].rstrip()
        values = line.split("; ")

        for value in values:
            v = value.split(" ")
            assert len(v) == 3

            if v[0] not in data:
                data[v[0]] = []
                units[v[0]] = v[2]

            data[v[0]].append(float(v[1]))

    def timestamp_to_str(timestamp):
        from datetime import datetime

        return datetime.fromtimestamp(int(timestamp)).strftime("%y%m%d %R")

    xdate = [timestamp_to_str(ts) for ts in data["@"]]
    x = list(range(len(xdate)))


    def formatter(x, pos):
        try:
            return xdate[int(x)]
        except IndexError:
            return ""


    _, ax = pyplot.subplots()
    ax.xaxis.grid(linestyle="dashed")
    ax.yaxis.grid(linestyle="dashed")
    ax.set_xlabel("Datetime")
    ax.xaxis.set_major_locator(ticker.MaxNLocator(12))
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(formatter))

    data["pressure"] = numpy.array(data["pressure"]) / 1000
    units["pressure"] = "kPa"

    for label, value in data.items():
        if label != "@":
            ax.plot(x, value,
                    label="{}, {}".format(label, units[label]))

    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand",
                borderaxespad=0.)
    pyplot.show()
