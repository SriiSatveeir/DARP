import numpy as np

class traversal:
    def __init__(self, times):
        
        self.times = times
        self.max = max(times)
        self.min = min(times)
        self.avg = np.average(times)
        self.std = np.std(times)

    def __str__(self):
        result = ''
        for i, t in enumerate(self.times):
            result += f'   Robot {i+1}: {t:.2f} seconds\n'
        result += (
            f'Traversal Times: {self.times} seconds\n'
            f'Max (mission times): {self.max:.2f} seconds\n'
            f'Min (mission times): {self.min:.2f} seconds\n'
            f'Average: {self.avg:.2f} seconds\n'
            f'Standard Deviation traversal time: {self.std:.2f} seconds\n'
            )
        return result

