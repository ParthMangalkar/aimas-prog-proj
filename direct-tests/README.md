# Direct Tests

This folder contains precompiled Windows executables for quick smoke testing without building the project first.

## Contents

- `bin/searchclient_cpp.exe`
  Precompiled C++ client.
- `bin/server_cpp.exe`
  Precompiled C++ server.

Testing levels are available directly under [`../levels`](C:\Users\Lenovo\Desktop\semester-2\MAS\new_cpp\aimas-warmup-assignment\levels).
The original Java server is kept under [`../misc/server.jar`](C:\Users\Lenovo\Desktop\semester-2\MAS\new_cpp\aimas-warmup-assignment\misc\server.jar).

## Test 1: C++ Client With Original Java Server

```powershell
java -jar "..\misc\server.jar" -l "..\levels\MAExample.lvl" -c ".\bin\searchclient_cpp.exe -bfs" -t 180
```

Expected result:

- the client starts searching
- the Java server reports the level was solved

## Test 2: Full C++ Stack

```powershell
.\bin\server_cpp.exe -l "..\levels\MAExample.lvl" -c ".\bin\searchclient_cpp.exe -bfs" -t 180
```

Expected result:

- the C++ server prints the level and client name
- the client finds a plan
- the C++ server reports `Solved in 30 steps.`

## Test 3: GUI Playback

```powershell
.\bin\server_cpp.exe -l "..\levels\MAExample.lvl" -c ".\bin\searchclient_cpp.exe -bfs" -t 180 -g -s 250
```

Expected result:

- the level solves
- a GUI window opens
- `Space`, `Left`, `Right`, `Home`, and `End` control playback

## Test 4: Timeout Smoke Test

```powershell
.\bin\server_cpp.exe -l "..\levels\MAExample.lvl" -c ".\bin\searchclient_cpp.exe -bfs" -t 1
```

Expected result:

- the run completes quickly on this small level or times out cleanly
- the server does not hang indefinitely

## Test 5: Directory Test

```powershell
.\bin\server_cpp.exe -l "..\levels" -c ".\bin\searchclient_cpp.exe -bfs" -t 180
```

Use this to catch issues that do not show up on a single level.

## Notes

- These binaries were built for Windows x64.
- The Java `-Xmx` option does not apply to `searchclient_cpp.exe`, since it is a native C++ executable.
