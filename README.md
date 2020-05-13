# SmallKat Dynamics and Balancing

This repo contains an implementation of `IDriveEngine` that enables SmallKat to walk around while maintaining balance.

## Balancing Algorithm

The jog widget (or a preprogrammed walk command) specify a walking velocity. This velocity forms the base of the step SmallKat will take. Added to this velocity is a closed-loop term used to maintain balance; this term proportionally controlled with feedback from the tilt angle of SmallKat's body.

The balance algorithm is implemented as a state machine that tracks whether SmallKat is not falling over, starting to fall over, or has fallen over.
- If SmallKat is not falling over (the body has not been rotated too far away from upright), then walk motions are performed without any balance corrections.
- If SmallKat is falling over (the body has been rotated away from upright but not too far away), then walk motions are combined with balance corrections. The balance correction controls both the step transform and the step height proportionally to the tilt of the body. As SmallKat falls over, longer and higher steps are required to maintain balance.
- If SmallKat has fallen over (the body has been rotated too far away from upright to recover), then all walk motions are stopped until SmallKat is returned to the upright position.

### Failures

- In developing the balance algorithm, I tried to use feed-forward control with the tilt rate term to increase the responsiveness of the balance controller. The main idea was to use the tilt rate to estimate the required feet positions `10ms` in the future to compensate for various time delays in the system. Ultimately, this made the controller less stable, so I removed the feed-forward terms. I think the instability came from having too long a time delay in the velocity control loop; maybe a much lower latency controller (implemented on the microcontroller directly, I imagine a `1kHz` loop with a negligible time delay) could work.
- I was unable to get SmallKat to recover from having fallen over (i.e., pick itself back up). It's important to note that I did my experiments with the Luna model, which features long legs and a long tail. I suspect that if the servos were slightly more powerful and the tail was slightly longer, then I would have been able to get SmallKat to recover.

## Progress Update Videos
- (Update 1)[https://www.youtube.com/watch?v=ehAgk_eAuPM]
- (Update 2)[https://www.youtube.com/watch?v=DoGOCxrBovg]
- (Update 3)[https://www.youtube.com/watch?v=efuPYoOV1Jk]
- (Update 4 Part 1)[https://www.youtube.com/watch?v=hzrionAnz3c]
- (Update 4 Part 2)[https://www.youtube.com/watch?v=59mjFYJ0z5w]
