        self.nav_pos = None
        self.nav_heading = None
        self.dr_pos = None
        self.dr_heading = None
        self.imu_heading = None
        
        self.nav_pos_last = None
        self.nav_heading_last = None
        self.dr_pos_last = None
        self.dr_heading_last = None
        self.imu_heading_last = None

    def update_sensors(self):
        self.nav_pos_last = self.nav_pos
        self.nav_heading_last = self.nav_heading
        self.dr_pos_last = self.dr_pos
        self.dr_heading_last = self.dr_heading
        self.imu_heading_last = self.imu_heading

        self.nav_pos = self.RH.nav_pos
        self.nav_heading = self.RH.navatt.heading if self.RH.navatt else None
        self.dr_pos = self.DR.dr_pos
        self.dr_heading = self.DR.dr_heading
        self.imu_heading = self.IH.imu_heading


    def valid_position(self, pos_last, pos_now, hz): # not used yet
        # threshold : under 120[km/h]
        diff = ((pos_now[0]-pos_last[0])**2 + (pos_now[1]-pos_last[1])**2)**0.5
        result = diff*hz < 120/3.6
        
        return result

    def valid_hdg(self, hdg_last, hdg_now, hz): # not used yet
        # threshold : under 100[deg/s]
        val = abs(hdg_last - hdg_now)
        diff = min(val, 360 - val)
        result = diff*hz < 5
        
        return result
        
    def heading_postprocess(self):
        if self.nav_heading - self.nav_heading_last < -355:
            self.nav_cw_cnt -= 1
        if self.nav_heading - self.nav_heading_last > 355:
            self.nav_cw_cnt += 1
        self.p_nav_heading = self.nav_heading - self.cw_cnt * 360

        if self.imu_heading - self.imu_heading_last < -355:
            self.imu_cw_cnt -= 1
        if self.imu_heading - self.imu_heading_last > 355:
            self.imu_cw_cnt += 1
        self.p_imu_heading = self.imu_heading - self.cw_cnt * 360

        if self.dr_heading - self.dr_heading_last < -355:
            self.dr_cw_cnt -= 1
        if self.dr_heading - self.dr_heading_last > 355:
            self.dr_cw_cnt += 1
        self.p_dr_heading = self.dr_heading - self.cw_cnt * 360