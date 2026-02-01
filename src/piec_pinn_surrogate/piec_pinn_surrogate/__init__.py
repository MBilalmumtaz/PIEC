# __init__.py
# Can be empty or have minimal imports
from .pinn_service import PINNService
from .pinn_model import PhysicsInformedPINN

__all__ = ['PINNService', 'PhysicsInformedPINN']
