# ONE Simulator - Enhanced DTN Visualizer & Analytics Platform

An enhanced and modernized version of **The ONE (Opportunistic Network Environment)** simulator, specialized for Delay Tolerant Network (DTN) algorithm research, routing evaluation, and real-time visualization.

This repository features a fully modernized Java Swing graphical interface, interactive metrics dashboard, real-time hop tracking with packet status categorization (`FORWARDED`, `DUPLICATE`, `DELIVERED`, `ABORTED`), instant speed controls, and an integrated event selection bar.

---

## 🌟 Key Features & Visual Enhancements

| Feature | Description |
|---|---|
| **📊 Live Metrics Dashboard** | Real-time tracking of message creation, delivery count, delivery ratio (%), average/min/max latency, average hop count, active connection links, and overhead ratio. |
| **⚡ Real-Time Path Hops Tab** | Automatically logs every forwarding hop in real-time with visual cards, node routes (`Node p0 ➔ Node c4`), timestamps, and color-coded status badges (`FORWARDED`, `DUPLICATE`, `DELIVERED`, `ABORTED`). Clicking any card centers the map on that transfer. |
| **📄 Full-Width Event Log & Selections** | Clean log rows with direct **Trace Path** action buttons and a full-width integrated top **Selections Bar** (`LINK UP`, `LINK DOWN`, `CREATED`, `FORWARDED`, `DUPLICATE`, `DELIVERED`, `ABORTED`, `DROPPED`, `REMOVED`) that eliminates the need to resize or adjust window splitters. |
| **🗺️ Visual Hop-by-Hop Route Tracing** | Renders high-contrast directional blue arrows with glowing halos, numbered waypoint badges, and dynamic transfer beams on the map canvas. |
| **⏩ Speed Controller Toolbar** | Quick multiplier buttons (`0.25x`, `0.5x`, `1x`, `2x`, `5x`, `10x`) to manage simulation speed on the fly. |
| **💾 One-Click Event Log Persistence** | Easily export complete timestamped event traces to formatted text files in the `reports/` directory. |
| **🎨 Unified Light Theme** | All panels (Metrics, Node Chooser, Path Hops, Log, and Canvas) share a clean, high-contrast, professional design. |

---

## 🛠️ Prerequisites

- **Java Development Kit (JDK)**: Java 11, 17, or 21 (Temurin / OpenJDK / Oracle JDK).
  - Verify with: `java -version` and `javac -version`
- **Operating System**: Windows, Linux, or macOS.
- **Git** (for version control).

---

## 🚀 How to Compile and Run

### 1. Compile the Project

- **Windows**:
  ```cmd
  compile.bat
  ```
  *or in PowerShell:*
  ```powershell
  .\compile.bat
  ```

- **Linux / macOS**:
  ```bash
  chmod +x compile.sh one.sh
  ./compile.sh
  ```

The compiled classes are generated in the `target/` directory with all dependencies (`lib/ECLA.jar`, `lib/DTNConsoleConnection.jar`) linked.

---

### 2. Run with Graphical User Interface (GUI)

- **Run Single Message Trace Scenario (Recommended for Hop Analysis)**:
  ```powershell
  .\one.bat SingleMessagePath_settings.txt
  ```

- **Run Standard Multi-Node Helsinki Scenario**:
  ```powershell
  .\one.bat default_settings.txt
  ```

- **Run Improved PRoPHET Scenario**:
  ```powershell
  .\one.bat ImprovedProphet_settings.txt
  ```

---

### 3. Run in Batch / Headless Mode (No GUI)

For long benchmark simulations and report generation without rendering the UI:
```powershell
.\one.bat -b 1 default_settings.txt
```
*(The `-b 1` flag runs 1 batch iteration and writes all output statistics directly to `reports/`)*.

---

## ⚙️ How to Configure & Customize Simulations

All simulation parameters in ONE are controlled via declarative configuration files (e.g. `SingleMessagePath_settings.txt` or `default_settings.txt`).

### 1. Modifying the Number of Nodes and Groups

Nodes in ONE belong to **Host Groups**. Each group can have its own speed, movement model, buffer size, radio interface, and router.

In your settings file:
```properties
# Total number of host groups
Scenario.nrofHostGroups = 2

# --------------------------------------------------
# Group 1: Pedestrians (IDs: 0, 1, 2, 3 -> p0, p1, p2, p3)
# --------------------------------------------------
Group1.groupID = p
Group1.nrofHosts = 4
Group1.speed = 0.5, 1.5           # Walking speed in m/s (min, max)
Group1.waitTime = 0, 20           # Pause time at waypoints (seconds)
Group1.bufferSize = 50M           # Storage capacity (50 Megabytes)

# --------------------------------------------------
# Group 2: Vehicles / Cars (IDs: 4, 5 -> c4, c5)
# --------------------------------------------------
Group2.groupID = c
Group2.nrofHosts = 2
Group2.speed = 2.7, 5.0           # Vehicle speed in m/s (min, max)
Group2.waitTime = 0, 10
Group2.bufferSize = 100M
```

> **Host ID Numbering Rule**: Host IDs are zero-indexed sequentially across groups.
> - If `Group1` has 4 hosts, they get host IDs `0..3` (`p0` to `p3`).
> - `Group2` hosts will automatically get host IDs `4..5` (`c4` and `c5`).

---

### 2. Configuring Message Generation (Source & Destination)

Message creation is driven by `MessageEventGenerator`:

```properties
Events.nrof = 1
Events1.class = MessageEventGenerator

# Message creation interval: (min, max) in simulation seconds
Events1.interval = 100, 100

# Creation time window: (start_time, end_time)
Events1.time = 0, 1000

# Message payload size (bytes or k/M)
Events1.size = 500k, 500k

# SOURCE HOST RANGE [min, max] (inclusive min, exclusive max)
# Example: hosts = 0, 1 means Source is node ID 0 (p0)
Events1.hosts = 0, 1

# DESTINATION HOST RANGE [min, max]
# Example: tohosts = 5, 6 means Destination is node ID 5 (c5)
Events1.tohosts = 5, 6

# Message ID prefix (e.g. M1, M2...)
Events1.prefix = M
```

To configure multiple message flows between random hosts across the entire network:
```properties
Events1.hosts = 0, 6      # Any node from 0 to 5 can be a source
Events1.tohosts = 0, 6    # Any node from 0 to 5 can be a destination
```

---

### 3. Updating Routing Protocols & Message Forwarding

#### A. Selecting a Built-In or Improved Router
In the group settings, set the router class:
```properties
# Available routers:
# - DiMaxPropRouter
# - ImprovedProphetRouter
# - ProphetRouter
# - EpidemicRouter
# - SprayAndWaitRouter
# - MaxPropRouter
# - DirectDeliveryRouter
Group.router = DiMaxPropRouter
```

#### B. Tuning Forwarding & Routing Parameters
For PRoPHET / Improved PRoPHET:
```properties
Group.maxForwards = 5             # Maximum number of times a node can forward a copy
Group.maxHops = 5                 # Maximum hop limit (Time-to-Live / Hop Count)

Group.pInit = 0.75                # Initial encounter predictability P_init
Group.gamma = 0.98                # Aging factor
Group.beta = 0.25                 # Transitivity scaling factor
Group.secondsInTimeUnit = 30      # Time unit for predictability aging
```

For Spray and Wait:
```properties
Group.router = SprayAndWaitRouter
SprayAndWaitRouter.nrofCopies = 6 # Initial L copies distributed
SprayAndWaitRouter.binaryMode = true
```

#### C. Writing Your Own Custom Routing Algorithm
To create a custom routing protocol:
1. Create a new Java class in `src/routing/MyCustomRouter.java`.
2. Extend `ActiveRouter` (or `MessageRouter`):
   ```java
   package routing;
   
   import core.Connection;
   import core.DTNHost;
   import core.Message;
   import core.Settings;
   
   public class MyCustomRouter extends ActiveRouter {
   
       public MyCustomRouter(Settings s) {
           super(s);
           // Read your custom parameters from settings file
       }
   
       protected MyCustomRouter(MyCustomRouter r) {
           super(r);
       }
   
       @Override
       public void update() {
           super.update();
           if (!canStartTransfer() || isTransferring()) {
               return; // Node is busy or radio channel unavailable
           }
   
           // Implement your forwarding logic:
           // Iterate over active connections and decide which messages to send
           for (Connection con : getConnections()) {
               DTNHost other = con.getOtherNode(getHost());
               // Your decision logic (e.g., utility function, encounter metric)
           }
       }
   
       @Override
       public MessageRouter replicate() {
           return new MyCustomRouter(this);
       }
   }
   ```
3. In your configuration file, simply set:
   ```properties
   Group.router = MyCustomRouter
   ```
4. Recompile with `compile.bat`.

---

### 4. Adjusting Simulation Area, Movement & Radios

#### Simulation Area & Clock
```properties
Scenario.name = MyCustomExperiment
Scenario.endTime = 2000           # Total simulation time in seconds
Scenario.updateInterval = 0.1     # Simulation resolution step (0.1s)
MovementModel.worldSize = 300, 300# Map width x height in meters
```

#### Radio Interface Range & Speed
```properties
Scenario.interfaces = Interface1
Interface1.type = SimpleBroadcastInterface
Interface1.transmitRange = 100    # Range in meters (nodes connect within 100m)
Interface1.transmitSpeed = 250k   # Speed (e.g. 250 kB/s = 2 Mbps)
```

#### Movement Models
```properties
# Movement models available:
# - RandomWaypoint
# - ShortestPathMapBasedMovement
# - MapRouteMovement
# - LinearMovement
Group1.movementModel = RandomWaypoint
```

---

### 5. Configuring Output Reports

ONE includes built-in reporting mechanisms that automatically output to text files when the simulation finishes:

```properties
Report.nrofReports = 2
Report.reportDir = reports/

Report.report1 = SingleMessagePathReport   # Hop-by-hop detailed trace of target message
Report.report2 = EventLogReport            # Full connection & message event log
# Report.report3 = MessageStatsReport      # High-level delivery stats & latency summary
# Report.report4 = ContactTimesReport      # Contact duration distribution
```

Generated reports are saved in `reports/<Scenario.name>_<ReportName>.txt`.

---

## 📁 Repository Directory Structure

```text
├── src/
│   ├── core/                  # Engine core: DTNHost, Message, SimClock, World, Settings
│   ├── gui/                   # Graphical user interface:
│   │   ├── DTNSimGUI.java     # Main GUI controller
│   │   ├── MainWindow.java    # Layout manager with tabbed analytics
│   │   ├── MetricsPanel.java  # Real-time dashboard statistics
│   │   ├── PathHopsPanel.java # Real-time hop cards (Forwarded / Duplicate / Delivered)
│   │   ├── EventLogPanel.java # Unified event log with integrated selections bar
│   │   ├── GUIControls.java   # Playback toolbar with 0.25x-10x speed multipliers
│   │   ├── InfoPanel.java     # Node & message detailed inspection
│   │   ├── NodeChooser.java   # Node filtering & selection
│   │   └── playfield/         # Map canvas, NodeGraphic, MessageGraphic, MessagePathGraphic
│   ├── movement/              # Movement models (RandomWaypoint, MapBased, Evening, etc.)
│   ├── report/                # Statistical reports (SingleMessagePathReport, MessageStats, etc.)
│   └── routing/               # Routers (ImprovedProphetRouter, Prophet, Epidemic, SprayAndWait)
├── lib/                       # External libraries (ECLA.jar, DTNConsoleConnection.jar)
├── data/                      # Map data files (WKT / roads / POIs)
├── reports/                   # Output directory for simulation logs and reports
├── compile.bat                # Windows build script
├── compile.sh                 # Linux/macOS build script
├── one.bat                    # Windows run launcher with automatic JDK detection
├── one.sh                     # Linux/macOS run launcher
├── SingleMessagePath_settings.txt # Single message hop trace scenario configuration
├── default_settings.txt       # Standard multi-node scenario configuration
└── README.md                  # Complete documentation and user guide
```

---

## 📜 License

This project is licensed under the **GNU General Public License v3.0 (GPLv3)**. See [LICENSE.txt](LICENSE.txt) for details.
