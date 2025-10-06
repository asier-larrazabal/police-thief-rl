Activate venvs

# Class computers
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process

mlagents-learn training\configs\car_agent.yaml --run-id=run43 --train