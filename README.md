# X-Label

Draws info about AI aircraft on the screen (on top of everything else).

![Screenshot](info/scrot1.png)

The box shows the AI aircraft number, the distance in kilometers, or
in meters if less than 5 kms, and the altitude difference with your
aircraft. The last symbol shows the vertical speed direction of the
target plane.

Can display distance to points of interest in a similar manner. This slows down X-plane a lot.

Can put a smoke emitting object at the nearest airport (needs to be defined in the configuration file).

Distance can be shown in metric or imperial units.

Possibility to warp the user aircraft to the first AI aircraft, copying position, speed and orientation.

# Usage

This plugin has no menu or user interface. There are eleven commands
you can assign to a keyboard key or joystick button. Search for
"durian/xlabel" in the keyboard assignment menu in X-Plane.

 - durian/xlabel/toggle_ac_label: shows the AI aircraft labels
 - durian/xlabel/toggle_ap_label: shows the POI labels
 - durian/xlabel/toggle_ap_smoker: puts smoke at the nearest airport
 - durian/xlabel/toggle_units: switch between metric/imperial
 - durian/xlabel/warp_to_next_ai: Warp user aicraft to next AI aircraft
 - durian/xlabel/warp_to_prev_ai: Warp user aircraft to previous AI aircraft
 - durian/xlabel/warp_to_closest_ai: Warp user aircraft to closest AI aircraft
 - durian/xlabel/max_shown_inc: Show more labels (one)
 - durian/xlabel/max_shown_dec: Show fewer labels (one)
 - durian/xlabel/max_dist_inc: Increase label viz distance by 10 kms
 - durian/xlabel/max_dist_dec: Decrease label viz distance by 10 kms

## POIs

The following shows a screenshot using a database with Swedish cities.

![Screenshot](info/scrot5.png)

Note that if you increase the number and distance, the view becomes cluttered.

![Screenshot](info/scrot6.png)

## Kitchensink

The following screen shot shows the labels for AI aircraft, POIs and a smoke marker on the airport.

![Screenshot](info/scrot4.jpg)

## Installing POIs

In `Output/preferences/`, `xlabel_pois.txt`.

```
# LAT      LON      ALT  DIST   LABEL
56.29210, 12.85447, 0,   10000, ESTA
56.33825, 12.89543, 0,    5000, Margretetorp
```

The label is shown when your aircraft is closed than `DIST` (meters). The `DIST` can be modified
with the label distance commands mention earlier.

## Smoker

In `Output/preferences/`, `xlabel.obj` and `xlabel.pss`.

## Note

Showing too many labels kills your framerate.
