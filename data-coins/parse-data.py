#!/usr/bin/python
fichier = ['courbe-data.txt', 'courbe-data-2.txt', 'courbe-data-4.txt', 'courbe-data-5.txt', 'courbe-data-8.txt', 'courbe-data-9.txt', 'courbe-data-10.txt', 'courbe-data-11.txt', 'courbe-data-12.txt']
data = []
for j, f in enumerate(fichier):
    fichier = open(f, 'r')
    cf = fichier.read()
    lines = cf.split("\n")
    i = 0
    debut = len(lines)*0.3
    fin = len(lines)*0.9
    for k, line in enumerate(lines):
        i += 1
        if i > debut and i < fin:
            infos = line.split(" ")
            try:
                if float(infos[1]) >= 160 and float(infos[3]) >= 0:
                    data.append(infos[1] + ',' + infos[2] + ',' + infos[3])
            except ValueError:
                continue

print "\n".join(data)
