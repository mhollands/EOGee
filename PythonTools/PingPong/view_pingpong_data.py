import numpy as np
import matplotlib.pyplot as plt

file = "pingpong_data_1606701148-094679.npy"
ref_point = -0.9
peak_point = 0.9

d = np.load(file, allow_pickle=True).item()

# Take data from EOG usb data stream and convert to ADC and DAC samples
def parse_data_stream(data):
	eogdata = []
	dacdata = []
	[eogdata.append(x & 0x0FFF) for x in data if (x >> 12) == 0x8]
	[dacdata.append(x & 0x0FFF) for x in data if (x >> 12) == 0x4]
	return eogdata, dacdata

# Given parameters for a DC transition, create data series to plot
def dc_transition_parameters_to_time_series(y0,y1, t1, t2, length):
	p1 = np.ones(t1)*y0
	if(y1 != y0 and t1 != t2):
		p2 = np.linspace(y0,y1,t2-t1+1)
	else:
		if(t1 != t2):
			p2 = np.ones(t2-t1+1)*y0
		else:
			p2 = [y0]
	p3 = np.ones(length-t2-1)*y1
	p = np.concatenate([p1,p2,p3])
	if(len(p) != length):
		print("length of series does not match expected value")
	return p

# Given some data, fit it to a DC transition curve
def fit_dc_tansition(data):
	l = len(data)

	# Make an intial guess for the DC transition parameters
	t1 = int(3*l/7)
	t2 = int(4*l/7)
	y0 = data[0]
	y1 = data[-1]

	def error_function(data, p):
		y0, y1, t1, t2 = p
		t1 = int(t1)
		t2 = int(t2)
		return np.sum((data - dc_transition_parameters_to_time_series(y0,y1, t1, t2, len(data)))**2)

	# p is our current guess
	p = [y0, y1, t1, t2]
	# Do multiple iterations of improvement
	n_iterations = 100
	for i in range(n_iterations):
		improved=False
		# Improve each of the parameters individually
		for tune_p in [0,1,2,3]:
			# Define the limits that we will search over for each parameter
			# y0 and y1 must be between the min and max of the data
			# t1 must be between 0 and t2
			# t2 must be between t1 and the length of the data
			lims = [[int(np.min(data)), int(np.max(data))], [int(np.min(data)), int(np.max(data))], [0, p[3]], [p[2], len(data)]]
			# Calculate our current error
			e0 = error_function(data, p)
			# Take a copy of our guess and sweep over the allowable range
			p_new = p.copy()
			for pp in range(lims[tune_p][0],lims[tune_p][1]):
				p_new[tune_p] = pp
				# Calculate the new error at the new guess
				e_new = error_function(data, p_new)
				# If it beats our old guess, replace the old guess with our new guess
				if(e_new < e0):
					e0 = e_new
					p = p_new.copy()
					improved = True
		# If the error did not improve on the previous iteration, we don't need to do any more iterations
		if(not improved):
			break

	[y0, y1, t1, t2] = p
	return [y0, y1, t1, t2]

# Given a list of stable sections, produce a time series data plot
def stable_sections_to_time_series(stable_sections):
	# Find the last end point of the stable sections to find out how long the data series needs to be
	length = np.max([x[3] for x in stable_sections])+1
	data = np.ones(length)*np.nan
	for i in range(len(stable_sections)):
		# For each section set the data points to the correct value
		section = stable_sections[i]
		start = section[2]
		end = section[3]
		y = section[0]
		data[start:end+1] = y
		# For all the but the first section insert the transition from the previous section
		if(i > 0):
			prev_section = stable_sections[i-1]
			prev_end = prev_section[3]
			prev_y = prev_section[0]
			data[prev_end:start+1] = np.linspace(prev_y, y, start-prev_end+1)

	# Extend the first stable section back to the start
	start_of_first_line = np.min([x[2] for x in stable_sections])
	data[:start_of_first_line] = data[start_of_first_line]
	return data

# Split data into dac data and eogdata
data = [parse_data_stream(x) for x in d["data"]]
eogdata = [x[0] for x in data]
dacdata = [x[1] for x in data]

# Get waypoints
points = d["points"]
xwaypoints = [p[0] for p in points]
# Save sample number for each waypoint
waypoints_adc_index = np.cumsum([len(x) for x in eogdata])
# Get a continuous array of xtarget
xtarget = np.concatenate([xwaypoints[i]*np.ones(len(eogdata[i])) for i in range(len(eogdata))])
# Get transition points
xtarget_transitions = [i for i in range(len(xtarget)-1) if np.diff(xtarget)[i] != 0 or i == 0]

# Flatten data arrays
eogdata = np.concatenate(eogdata)
dacdata = np.concatenate(dacdata)

# Make sure ADC and DAC data are same length
eogdata = eogdata[:min([len(eogdata), len(dacdata)])]
dacdata = dacdata[:min([len(eogdata), len(dacdata)])]

# Combine ADC and DAC data
dac_adc_counts = 22*1.5;
eogdata = eogdata - (dacdata - 2048) * dac_adc_counts

# Smooth the data
eogdata = np.convolve(eogdata, np.ones(100)/100, mode="valid")

# Plot
fig,ax1 = plt.subplots(1,1)
ax2 = ax1.twinx()

#Find the midpoint of each target sections
xtarget_midpoints = np.convolve(xtarget_transitions, [0.5, 0.5], mode="valid").astype(int)
# Fit DC transitions to each midpoint-midpoint
dc_transitions = []
for i in range(1, len(xtarget_midpoints)):
	print("Fitting section {0}/{1}".format(i, len(xtarget_midpoints)))
	xtransition_eogdata = eogdata[xtarget_midpoints[i-1]:xtarget_midpoints[i]]
	[y0, y1, t1, t2] = fit_dc_tansition(xtransition_eogdata)
	dc_transitions.append([y0,y1,xtarget_midpoints[i-1], xtarget_midpoints[i-1]+t1, xtarget_midpoints[i-1]+t2, xtarget_midpoints[i]])
	xplt = range(xtarget_midpoints[i-1],xtarget_midpoints[i])

	# Plot DC transition and the data it approximates
	ax1.plot(xplt, eogdata[xtarget_midpoints[i-1]:xtarget_midpoints[i]], color="blue")
	ax1.plot(xplt,dc_transition_parameters_to_time_series(y0,y1, t1, t2, len(xtransition_eogdata)), color="orange")

# Plot the waypoints on a separate axis
ax2.scatter(waypoints_adc_index, xwaypoints, color="red")

# Convert DC transitions to list of stable regions
stable_sections = []
for i in range(0, len(dc_transitions)+1):
	print("Merging section {0}/{1}".format(i+1,len(dc_transitions)+1))
	# For each pair of dc_transitions we have two sections of line than need to be joined - the pre and the post
	# We take the weighted average of the two sections of line to find the vlaue of the average line
	# There are exceptions for the first and last section
	pre_section_y = dc_transitions[i-1][1] if i > 0 else 0
	post_section_y = dc_transitions[i][0] if i < len(dc_transitions) else 0
	pre_section_length = dc_transitions[i-1][5]-dc_transitions[i-1][4] if i > 0 else 0
	post_section_length = dc_transitions[i][3]-dc_transitions[i][2] if i < len(dc_transitions) else 0
	pre_section_start = dc_transitions[i-1][4]  if i > 0 else dc_transitions[i][2]
	post_section_end = dc_transitions[i][3] if i < len(dc_transitions) else dc_transitions[i-1][5]
	y = (pre_section_y*pre_section_length + post_section_y*post_section_length)/(pre_section_length+post_section_length)
	# The target is the waypoint at the start of the post-section
	post_section_start = dc_transitions[i][2] if i < len(dc_transitions) else dc_transitions[i-1][2]
	post_section_target = xtarget[post_section_start]
	start = pre_section_start
	end = post_section_end
	target = post_section_target
	stable_sections.append([y, target, start, end])

# Plot the data series from all these stable sections
ax1.plot(stable_sections_to_time_series(stable_sections))

# Take only the stable sections where the user was looking at the reference point
ref_stable_sections = [x for x in stable_sections if x[1] == ref_point]
# Construct drift time series
drift_data = stable_sections_to_time_series(ref_stable_sections)
ax1.plot(drift_data)

# Subtract the drift from each stable section
driftless_stable_sections = []
for i in range(len(stable_sections)):
	[y, target, start, end] = stable_sections[i]
	# Use the drift at the midpoint of each stable section
	midpoint = np.min([int((start + end)/2), len(drift_data)-1])
	drift = drift_data[midpoint]
	driftless_y = y - drift
	driftless_stable_sections.append([driftless_y, target, start, end])

# Take only the stable sections where the user was looking at the peak point
peak_stable_sections = [x for x in driftless_stable_sections if x[1] == peak_point]
# Construct gain time series
gain_data = stable_sections_to_time_series(peak_stable_sections)

# Divide stable section by the gain
normalised_driftless_stable_sections = []
for i in range(len(driftless_stable_sections)):
	[y, target, start, end] = driftless_stable_sections[i]
	# Use the gain at the midpoint of each stable section
	midpoint = np.min([int((start + end)/2), len(gain_data)-1])
	gain = gain_data[midpoint]
	normalised_y = y/gain
	normalised_driftless_stable_sections.append([normalised_y, target, start, end])

normalised_driftless_data = (peak_point - ref_point) * stable_sections_to_time_series(normalised_driftless_stable_sections) + ref_point
# Plot the data series from all these stable sections
normalised_fig, normalised_ax = plt.subplots(1,1)
normalised_ax.plot(normalised_driftless_data)

plt.show(block=False)