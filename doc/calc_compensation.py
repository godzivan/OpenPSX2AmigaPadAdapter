#!/usr/bin/env python

import matplotlib.pyplot as pyplot

RTGTMAX = 470000

# AD5242BR1M
R_WIPER = 60
R_AB = 1000000

RPAR = 1000000
STEPS = 256


def par (a, b):
	if a == 0 or b == 0:
		return 0
	return 1 / (1 / float (a) + 1 / float (b))

x = range (0, STEPS)

rdesired = [float (RTGTMAX) * i / STEPS for i in x]
pyplot.plot (x, rdesired, label = 'Desired Resistance Values', color = 'b')

# Raw values available from AD5242BR1M
rraw = []
for i in x:
	r = (256 - i) / 256.0 * float (R_AB) + R_WIPER
	rraw.append (r)
pyplot.plot (x, rraw, label = 'Available Raw Resistance Values', color = 'y')

# Rwb
# ~ rraw_wb = []
# ~ for i in x:
	# ~ r = i / 256.0 * float (R_AB) + R_WIPER
	# ~ rraw_wb.append (r)
# ~ pyplot.plot (x, rraw_wb, label = 'Available Rwb Resistance Values')

ravail = []
for r in rraw:
	ravail.append (par (r, RPAR))
pyplot.plot (x, ravail, label = 'Available Paralleled Resistance Values', color = 'r')

rcompensated = []
rcompensatedn = []
for i in x:
	rtgt = float (RTGTMAX) * i / STEPS
	print "Target : %u" % rtgt

	mindiff = RTGTMAX
	n = -1
	for j, r in enumerate (ravail):
		diff = abs (rtgt - r)
		if diff < mindiff:
			mindiff = diff
			n = j

	rcompensated.append (ravail[n])
	rcompensatedn.append (n)
	print "Closest value: %u (%u), diff = %f (%u%%)" % (ravail[n], n, mindiff, (0 if rtgt == 0 else int (mindiff * 100.0 / rtgt)))
	print "-" * 80

pyplot.plot (x, rcompensated, label = 'Compensated Resistance Values', color = 'g')

print "-" * 80
print "const byte rcompensated[%u] = {\n\t" % len (rcompensatedn),
for i, r in enumerate (rcompensatedn):
	print "%u, " % r,
	if (i + 1) % 8 == 0:
		print "\n\t",
print "\r};"

# ~ pyplot.title('mindmajix')
pyplot.ylabel ('Resistance')
pyplot.xlabel ('Value')
pyplot.legend ()
pyplot.grid ()
pyplot.show ()
