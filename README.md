# Unity version
Unity 6000.0.41f1

# Python version
3.10.9

# Create environment
- Create .venv environment with Python version 3.10.9
- Install requirements from requirements.txt

# For activate .venv in Class Computers
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process

# For running the .yaml file (with new id)
mlagents-learn training\configs\car_agent.yaml --run-id=run43 --train