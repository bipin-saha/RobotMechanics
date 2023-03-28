import numpy as np

class Link:

    def __init__(self, name='', alpha=0, a=0, d=0, theta=0,
                 joint_type = 'revolute', inertia_matrix = None, cog = None, mass=0):
        
        self.name = name
        self.alpha = alpha
        self.a = a
        self.d = d
        self.theta = theta
        self.joint_type = joint_type
        self.inertia_matrix = inertia_matrix
        self.cog = cog
        self.mass = mass

    def transform(self, q):
        alpha = self._alpha
        a = self._a

        if self._joint_type == 'revolute':
            theta = q
            d = self._d
        else:
            theta = self._theta
            d = q

        transformation = np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
            [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
            [0, 0, 0, 1]
        ])

        return transformation
    
    @property
    def name(self):
        return self._name
    
    @name.setter
    def name(self, name):
        self._name = name

    @property
    def alpha(self):
        return self._alpha
    
    @alpha.setter
    def alpha(self, alpha):
        self._alpha = alpha

    @property
    def a(self):
        return self._a
    
    @a.setter
    def a(self, a):
        self._a = a

    @property
    def d(self):
        return self._d
    
    @d.setter
    def d(self, d):
        self._d = d

    @property
    def theta(self):
        return self._theta
    
    @theta.setter
    def theta(self, theta):
        self._theta = theta

    @property
    def joint_type(self):
        return self._joint_type
    
    @joint_type.setter
    def joint_type(self, joint_type):
        joint_types = ['revolute', 'prismatic']
        try:
            assert(joint_type in joint_types)
            self._joint_type = joint_type
        except AssertionError:
            raise(f"Joint type {joint_type} not defined")
        
    @property
    def inertia_matrix(self):
        return self._inertia_matrix
    
    @inertia_matrix.setter
    def inertia_matrix(self, inertia_matrix):
        try:
            assert(inertia_matrix.shape == (3,3))
            self._inertia_matrix = inertia_matrix
        except AssertionError:
            raise("Shape of inertia matrix is incorrect")
        
    @property
    def cog(self):
        return self._cog
    
    @cog.setter
    def cog(self, cog):
        try:
            assert(cog.shape == 3)
            self._cog = cog
        except AssertionError:
            raise("Shape of COG vector is incorrect")
        
    @property
    def mass(self):
        return self._mass
    
    @mass.setter
    def mass(self, mass):
        self._mass = mass
