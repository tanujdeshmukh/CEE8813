class Link:
    """
    Class for network links.
    """

    def __init__(self, network, tail, head, capacity = 99999, length = 99999, freeFlowTime = 99999, alpha = 0.15, beta = 4, speedLimit = 99999, toll = 0, linkType = 0):
        """
        Initializer for links; note default values for parameters if not specified.
        """
        self.network = network
        self.tail = tail
        self.head = head

        self.capacity = capacity 
        self.capacity_HDV = capacity #Capacity read from the .tntp file is the HDV capacity which will be directly consumed. This is Q_a_H
        self.length = length
        self.freeFlowTime = freeFlowTime
        self.alpha = alpha
        self.beta = beta
        self.speedLimit = speedLimit
        self.toll = toll
        self.linkType = linkType

        #The free flow speed used for calculating Q_a_E needs v_a so we need to define it.
        self.freeFlowSpeed = self.length / self.freeFlowTime

        # Derived capacity for EVs using the provided formula
        self.capacity_EV = self.calculateEVCapacity()
         

        # Set the intials Flows for HDVs and EVs
        self.flow_HDV = 0
        self.flow_EV = 0

        # Initial costs
        self.cost_HDV = self.calculateCost_HDV()
        self.cost_EV = self.calculateCost_EV()

    @property
    def HDV_Capacity(self):
        return self.capacity_HDV

    # Property for EV capacity
    @property
    def EV_Capacity(self):
        return self.capacity_EV
    
    def calculateCost_HDV(self):

        #Calculates the cost for HDVs based on their flow and the total flow on the link.

        vcRatio_HDV = self.flow_HDV / self.capacity_HDV
        vcRatio_EV = self.flow_EV / self.capacity_EV
        combined_vcRatio = vcRatio_HDV + vcRatio_EV

        if combined_vcRatio <= 0:
            return self.freeFlowTime
        
        LAMBDA = 1
        term1 = (1 + pow(combined_vcRatio, 4))
        term2 =  LAMBDA * combined_vcRatio

        travelTime_H = ((self.length / self.freeFlowSpeed) * (term1)) + term2
        return travelTime_H


    def calculateCost_EV(self):

        vcRatio_HDV = self.flow_HDV / self.capacity_HDV
        vcRatio_EV = self.flow_EV / self.capacity_EV
        combined_vcRatio = vcRatio_HDV + vcRatio_EV

        # Fall back for negative flows
        if combined_vcRatio <= 0:
            return self.freeFlowTime
        
        KAPPA_E = 0.4
        LAMBDA = 1
        term1 = (1 + pow(combined_vcRatio, 4))
        term2 = KAPPA_E * LAMBDA * combined_vcRatio

        travelTime_E = (self.length / self.freeFlowSpeed) * (term1) + term2
        return travelTime_E
    
    def calculateEVCapacity(self):
        """
        Calculates the capacity for EVs based on the capacity for HDVs using the provided formula.
        """
        del_t_H = 1.5  # Reaction time for HDVs in seconds
        del_t_E = 1    # Reaction time for EVs in seconds
        vehicle_length = 15  # Vehicle length in feet

        return self.capacity_HDV * ((self.freeFlowSpeed * del_t_H + vehicle_length) / (self.freeFlowSpeed * del_t_E + vehicle_length))
    
      

    def calculateBeckmannComponent_HDV(self):
        vcRatio_HDV = self.flow_HDV / self.capacity_HDV
        vcRatio_EV = self.flow_EV / self.capacity_EV
        combined_vcRatio = vcRatio_HDV + vcRatio_EV

        LAMBDA = 0.1
        term1 = (self.length / self.freeFlowSpeed) * self.flow_HDV
        term2 = (self.length / self.freeFlowSpeed) * (self.capacity_HDV / 5) * pow(combined_vcRatio, 5)
        term3 = LAMBDA * (self.capacity_HDV / 2) * pow(combined_vcRatio, 2)

        return term1 + term2 + term3

    def calculateBeckmannComponent_EV(self):
        vcRatio_HDV = self.flow_HDV / self.capacity_HDV
        vcRatio_EV = self.flow_EV / self.capacity_EV
        combined_vcRatio = vcRatio_HDV + vcRatio_EV

        LAMBDA = 0.1
        KAPPA_E = 0.4
        term1 = (self.length / self.freeFlowSpeed) * self.flow_EV
        term2 = (self.length / self.freeFlowSpeed) * (self.capacity_EV / 5) * pow(combined_vcRatio, 5)
        term3 = KAPPA_E * LAMBDA * (self.capacity_EV / 2) * pow(combined_vcRatio, 2)

        return term1 + term2 + term3

    def calculateBeckmannComponent(self):
        return self.calculateBeckmannComponent_HDV() + self.calculateBeckmannComponent_EV()
    
    def addFlow(self, flow_HDV=0, flow_EV=0):

        # Adds flow to the link for HDVs and EVs and updates the costs accordingly.
        self.flow_HDV += flow_HDV
        self.flow_EV += flow_EV
        self.updateCosts()
    
    def updateCosts(self):
        self.cost_HDV = self.calculateCost_HDV()
        self.cost_EV = self.calculateCost_EV()
