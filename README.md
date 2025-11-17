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



# Entrenamiento de Agentes con ML-Agents

Este proyecto utiliza **Unity ML-Agents** para entrenar agentes de IA en un entorno de Unity.

---

## Requisitos

- Python 3.10.x
- Unity Editor con el proyecto cargado
- Git (opcional, si clonas el repo)

---

## Instalación del entorno Python

1. Crear un entorno virtual:

```bash
python -m venv .venv_ml

.venv_ml\Scripts\Activate.ps1

source .venv_ml/bin/activate

pip install --upgrade pip
pip install mlagents==1.1.0 mlagents-envs==1.1.0
pip install torch==1.13.1+cpu torchvision==0.14.1+cpu torchaudio==0.13.1 --index-url https://download.pytorch.org/whl/cpu
pip install onnx==1.15.0 protobuf==3.20.3 numpy


mlagents-learn training/configs/car_agent.yaml --run-id=run1

---
