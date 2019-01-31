## Benchmark Results

| System | Camera | CUDA? | Overclocked? | Camera fetch | Camera calc | File calc |
| ------ | ------ | ----- | ------------ | ------------ | ----------- | --------- |
| Jetson | USB    | No    | No           | 56 ms        | 9 ms        | 3 ms      |
| Jetson | USB    | Yes   | No           | 50 ms        | 23 ms       | 16 ms     |
| Jetson | USB    | No    | Yes          | 64 ms        | 3 ms        | 4 ms      |
| Jetson | USB    | Yes   | Yes          | 60 ms        | 12 ms       | 12 ms     |
| Raspi  | USB    | No    | No           | 22 ms        | 10 ms       | 10 ms     |
| Raspi  | Raspi  | No    | No           |  |  |  |

### Notes
The camera fetch times are dependent on the brightness of the environment; the Jetson tests were done in a darker environment while the Raspi tests were done in a brighter environment.
