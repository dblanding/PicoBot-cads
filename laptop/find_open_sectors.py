sensor_D_offset = 15

class FindOpenSectors:
    """This class will analyze data from a robot turning in place and
    return a list of obstacle-free sectors. Each sector will be a dict
    containing:
        * distance to detected objects (if any) on each side of the sector
        * robot pose associated with the detection of those objects
        * the number of obstacle-free data points within the sector
    """
    def __init__(self):
        self.curr_sector = None
        self.sectors = []
        self.prev_pose = None
        self.prev_dist = None
        self.nmbr_open_reads = 0

    def process_next(self, data: dict):
        """Process next in a series of robot data during turn-in-place."""
        sensor_D_val = data.get('distances')[3]
        pose = data.get('pose')
        self.nmbr_open_reads += 1
        if not self.prev_dist:  # Initial reading
            if sensor_D_val > 8000:  # begin an open sector
                self.curr_sector = {
                    'first_dist': None,
                    'first_pose': pose,
                    }
        else:  # Subsequent reading
            if not self.curr_sector:  # not currently in an open sector
                if sensor_D_val > 8000:  # begin an open sector
                    self.nmbr_open_reads = 0
                    self.curr_sector = {
                        'first_dist': self.prev_dist,
                        'first_pose': self.prev_pose
                        }
                    pass
            else:  # currently in an open sector
                if sensor_D_val < 8000:  # end an open sector
                    self.curr_sector.update(
                        {'last_dist': sensor_D_val + sensor_D_offset,
                        'last_pose': pose,
                         'nmbr_reads_wide': self.nmbr_open_reads, }
                        )
                    self.sectors.append(self.curr_sector)
                    self.curr_sector = None
                    self.prev_pose = pose
                    self.prev_dist = sensor_D_val + sensor_D_offset
        self.prev_pose = pose
        self.prev_dist = sensor_D_val + sensor_D_offset

    def get_sectors(self, ):
        return self.sectors
            
        
