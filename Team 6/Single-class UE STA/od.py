class OD:
    def __init__(self, origin, destination, demand, percentage_HDV = 0, percentage_EV =0):
        """
        Initializes an OD pair with separate demands for HDVs and EVs.
        """
        self.origin = origin
        self.destination = destination
        self.demand = demand
        self.demand_HDV = demand * percentage_HDV
        self.demand_EV = demand * percentage_EV

    def setDemand(self, demand_HDV=0, demand_EV=0):
        """
        Sets the demand for both HDVs and EVs for this OD pair.
        """
        self.demand_HDV = demand_HDV
        self.demand_EV = demand_EV

    def getTotalDemand(self):
        """
        Returns the total demand (HDV + EV) for this OD pair.
        """
        return self.demand_HDV + self.demand_EV

