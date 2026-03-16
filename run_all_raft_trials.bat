@echo off
REM Complete Raft rerun - 5 trials per condition
REM Paper timing: t=10s fault injection

echo === Raft Clean (5 trials) ===
for /L %%i in (1,1,5) do (
    echo Trial %%i/5...
    python run_trial.py --controller raft --fault none --trial %%i
    timeout /t 5 /nobreak
)

echo === Raft Freeze Leader (5 trials) ===
for /L %%i in (1,1,5) do (
    echo Trial %%i/5...
    python run_trial.py --controller raft --fault freeze_leader --fault-time 10 --trial %%i
    timeout /t 5 /nobreak
)

echo === Raft Self-Injure (5 trials) ===
for /L %%i in (1,1,5) do (
    echo Trial %%i/5...
    python run_trial.py --controller raft --fault self_injure_leader --fault-time 10 --trial %%i
    timeout /t 5 /nobreak
)

echo === Raft Bad Leader (5 trials) ===
for /L %%i in (1,1,5) do (
    echo Trial %%i/5...
    python run_trial.py --controller raft --fault bad_leader --fault-time 10 --trial %%i
    timeout /t 5 /nobreak
)

echo === DONE ===
