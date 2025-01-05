import numpy as np
import config


class GPSTrajectory:
    def __init__(self, latitude, longitude, altitude):
        self.latitude = np.radians(latitude)
        self.latitude0 = self.latitude[config.index["stand_by"]]
        self.longitude = np.radians(longitude)
        self.longitude0 = self.longitude[config.index["stand_by"]]
        self.altitude = altitude
        self.pos = {"x": [], "y": [], "z": []}  # Position in observer frame

    def haversine(self, lat, lon, radius=6378*1000):
        """Calculate distance using haversine formula."""
        dlat = lat - self.latitude0
        dlon = lon - self.longitude0
        
        a = np.sin(dlat/2)**2 + np.cos(self.latitude0) * np.cos(lat) * np.sin(dlon/2)**2
        c = 2 * np.arcsin(np.sqrt(a))
        
        x = radius * np.cos(self.latitude0) * np.sin(dlon)
        y = radius * np.sin(dlat)
        return x, y

    def calculate_trajectory(self):
        for i in range(len(self.latitude)):
            x, y = self.haversine(self.latitude[i], self.longitude[i])
            self.pos['x'].append(x)
            self.pos['y'].append(y)
            self.pos['z'].append(self.altitude[i])

    def plot_trajectory(self, ax, color, label):
        ax.plot(self.pos['x'], self.pos['y'], self.pos['z'], color=color, label=label)
