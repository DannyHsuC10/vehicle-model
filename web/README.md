# Vehicle 14DOF Lab Web App

The web app runs the shared Python 14-DOF model and NumPy in a Pyodide Web Worker. It supports a 9.125 m radius skidpad, a 75 m acceleration run, and lap timing on custom tracks. Simulations and imported files stay in your browser. The first run needs access to jsDelivr to download the pinned Pyodide and NumPy versions.

## GitHub Pages

1. Push your changes to the GitHub repository.
2. Under **Settings → Pages → Build and deployment → Source**, select **GitHub Actions**.
3. Under **Actions → Deploy Vehicle14DOF web**, choose **Run workflow** on `main`. Subsequent changes to the web app or solver on `main` trigger a build and deployment automatically.
4. When deployment completes, open the site URL from the workflow's `github-pages` environment.

The workflow publishes `dist/vehicle14dof/`. The build contains the static site, solver source, LP03 defaults, a sample skidpad CSV, and the Pacejka license. The tire assets include the public demo tire and a downloadable zero-value TIR template. The browser uses any TIR file you import and includes it in the results ZIP. The desktop FSAE track, STL assets, historical telemetry, and test data are available in the repository.

## Local Preview

Run the following commands from the repository root, then open <http://127.0.0.1:8000/>. On Windows, use `.\.venv\Scripts\python.exe` if you already have a project virtual environment.

```powershell
python -m pip install -r Vehicle14DOFLab/requirements.txt
python -B web/build.py
python -m http.server 8000 --bind 127.0.0.1 --directory dist/vehicle14dof
```

## Inputs and Results

- The built-in skidpad has a radius of 9.125 m and the straight is 75 m long. Custom Track accepts a closed LINE/ARC track CSV. The downloadable skidpad example consists of two connected 180° arcs. Use meters for coordinates, radii, and lengths, degrees for start and end angles, and `Clockwise` or `Counterclockwise` for direction. Each segment must connect to the next to form a closed loop.
- **Vehicle Settings** covers body and center of gravity, powertrain and aerodynamics, tires and wheels, hydraulic brakes, and suspension. You can import a desktop suspension project or provide the files referenced by a vehicle JSON using the suspension, tire, motor, and aerodynamic upload fields. Values entered on the page override the source settings. Blank fields use the uploaded file or LP03 defaults.
- **Tires and Wheels** shares wheel rotational inertia, tire pressure, vertical stiffness, and vertical damping across all four wheels. Free radius, unsprung mass, static camber, and drive torque scale are set by axle. For imported files, the front-left wheel supplies the shared values. Each axle's left side supplies the other wheel settings, camber gain, and direct-acting suspension settings. The results ZIP contains the effective values.
- **Suspension and Geometry** covers front and rear roll center heights above the road (mm), anti geometry (%), and camber gain per 25 mm of compression. Roll center and anti geometry require load transfer to be enabled. Camber gain can take effect independently. Imported absolute lookup tables disable the associated fields. Enabled suspension channels accept a spring rate (N/mm) and fixed motion ratio. A spring-rate override adjusts the main spring while retaining any tender spring, or replaces an imported staged or force curve with a fixed rate. Direct-acting settings apply to both sides of the axle.
- Compression and rebound damping fields show the LP03 reference coefficients (N·s/m) and their speed thresholds. An imported curve shows its source values and units. Leave either field blank to keep the corresponding source curve. Enter a fixed coefficient to override it or enter `0` to disable damping in that direction. Settings apply to enabled channels.
- **Import Settings and Curves** offers downloadable templates for vehicle, suspension, and aerodynamic JSON files and for tire TIR files. Numeric values start at zero, switches start disabled, and format identifiers and file versions are retained. Fill in your parameters before importing. The sample motor CSV uses `vdc_V,rpm,T_shaft_Nm` for a multi-voltage envelope. Torque is in N·m at the motor shaft before reduction. Put aerodynamic coefficients at the top level of the JSON file and include `CL0` and `CD0`.
- The driving replay animates the car's body attitude, wheel rotation, and steering from telemetry. It also displays a G–G diagram, pedal inputs, and vehicle speed. Use the follow or overhead view, the playback timeline, and speeds from 0.25× to 4×.
- Below the replay you can view the speed-colored vehicle path, individual wheel telemetry, and run timing. The results ZIP includes the effective settings, inputs, telemetry CSV, timing CSV, and SVG charts.
