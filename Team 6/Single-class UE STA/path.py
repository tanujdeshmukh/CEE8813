class Path:

    def __init__(self, links, network, flow = 0):
        self.links = links
        self.network = network
        self.flow = flow
        self.updateCost()


    def calculateCost(self, vehicle_type='HDV'):
        total_cost = 0
        for link_id in self.links:
            link = self.network.link[link_id]
            if vehicle_type == 'HDV':
                total_cost += link.calculateCost_HDV()
            elif vehicle_type == 'EV':
                total_cost += link.calculateCost_EV()
        return total_cost
    

    def updateCost(self):
        self.cost_HDV = self.calculateCost('HDV')
        self.cost_EV = self.calculateCost('EV')

    def updateFlow(self, flow_HDV=0, flow_EV=0):
        self.flow_HDV += flow_HDV
        self.flow_EV += flow_EV
        self.updateCost()

    def calculateBeckmannComponent(self, vehicle_type='HDV'):
        beckmann_total = 0
        for link_id in self.links:
            link = self.network.link[link_id]
            if vehicle_type == 'HDV':
                beckmann_total += link.calculateBeckmannComponent_HDV()
            elif vehicle_type == 'EV':
                beckmann_total += link.calculateBeckmannComponent_EV()
        return beckmann_total
