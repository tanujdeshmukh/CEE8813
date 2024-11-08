from link import Link
from node import Node
from path import Path
from od import OD


import sys
import traceback
import utils
import csv

FRANK_WOLFE_STEPSIZE_PRECISION = 1e-7

class Network:
    """
    This is the class used for transportation networks.  It uses the following
    dictionaries to store the network; the keys are IDs for the network elements,
    and the values are objects of the relevant type:
        node -- network nodes; see node.py for description of this class
        link -- network links; see link.py for description of this class
        ODpair -- origin-destination pairs; see od.py
        path -- network paths; see path.py.  Paths are NOT automatically generated
                when the network is initialized (you probably wouldn't want this,
                the number of paths is exponential in network size.)
                
        The network topology is expressed both in links (through the tail and head
        nodes) and in nodes (enteringLinks and leavingLinks are Node attributes storing
        the IDs of entering and leaving links in a list).
                
        numNodes, numLinks -- self-explanatory
        numZones -- 
        firstThroughNode -- in the TNTP data format, transiting through nodes with
                            low IDs can be prohibited (typically for centroids; you
                            may not want vehicles to use these as "shortcuts").
                            When implementing shortest path or other routefinding,
                            you should prevent trips from using nodes with lower
                            IDs than firstThroughNode, unless it is the destination.
    """

    def __init__(self, networkFile="", demandFile=""):
        """
        Class initializer; if both a network file and demand file are specified,
        will read these files to fill the network data structure.
        """
        self.numNodes = 0
        self.numLinks = 0
        self.numZones = 0
        self.firstThroughNode = 0

        self.node = dict()
        self.link = dict()
        self.ODpair = dict()
        self.path = dict()

        if len(networkFile) > 0 and len(demandFile) > 0:
            self.readFromFiles(networkFile, demandFile)
            
    def formIncidenceMatrix(self):
        
        self.incidenceMatrix = dict()
        for i in self.node:
            self.incidenceMatrix[i] = dict()
            for ij in self.link:
                self.incidenceMatrix[i][ij] = 0
        
        for ij in self.link:
            self.incidenceMatrix[self.link[ij].tail][ij] = 1
            self.incidenceMatrix[self.link[ij].head][ij] = -1
    
        return self.incidenceMatrix
        
    
    def formAdjacencyMatrix(self):
    
        self.adjacencyMatrix = dict()
        for i in self.node:
            self.adjacencyMatrix[i] = dict()
            for j in self.node:
                self.adjacencyMatrix[i][j] = 0
            
        for ij in self.link:
            self.adjacencyMatrix[self.link[ij].tail][self.link[ij].head] = 1
    
        return self.adjacencyMatrix
    
    
    def calculateDegrees(self):
    
        self.degrees = {"indegree": dict(), "outdegree": dict()}
        for i in self.node:
            self.degrees["indegree"][i] = 0
            self.degrees["outdegree"][i] = 0
        
        for ij in self.link:
            self.degrees["indegree"][self.link[ij].head] += 1
            self.degrees["outdegree"][self.link[ij].tail] += 1
        
        return self.degrees
        
    def beckmannFunction(self):
        """
        This method evaluates the Beckmann function at the current link
        flows.
        """
        #In multi class you will now use a combination of beckman compenent for each of the link
        #This will have the EV compnents as well as the SO component.
        beckmann = 0
        for ij in self.link:
            beckmann += self.link[ij].calculateBeckmannComponent()  # This will now return the sum of EV and HDV

        return beckmann
        
    def shiftFlows(self, targetFlows_HDV, targetFlows_EV, alpha):
        for ij in self.link:
            # Compute the new flow for HDV
            self.link[ij].flow_HDV = (1 - alpha) * self.link[ij].flow_HDV + alpha * targetFlows_HDV[ij]
            
            # Compute the new flow for EV
            self.link[ij].flow_EV = (1 - alpha) * self.link[ij].flow_EV + alpha * targetFlows_EV[ij]
            
            # Update the link costs based on new flows
            self.link[ij].updateCosts()

    def shortestPath(self, origin, vehicleType='HDV'):
        print(f"\nCalculating Shortest Path from Origin {origin} for Vehicle Type: {vehicleType}")
        backlink = dict()
        cost = dict()

        for i in self.node:
            backlink[i] = utils.NO_PATH_EXISTS
            cost[i] = utils.INFINITY
        cost[origin] = 0

        scanList = [origin]

        while len(scanList) > 0:
            i = scanList.pop(0)  # Dequeue the first element
            print(f"\nExploring Node: {i} for {vehicleType}")
            for forwardLink in self.node[i].leavingLinks:
                followingNode = self.link[forwardLink].head

                # Choose the appropriate cost based on vehicle type
                if vehicleType == 'HDV':
                    tempCost = cost[i] + self.link[forwardLink].cost_HDV
                elif vehicleType == 'EV':
                    tempCost = cost[i] + self.link[forwardLink].cost_EV
                else:
                    raise Exception("Invalid vehicle type. Choose 'HDV' or 'EV'.")

                if tempCost < cost[followingNode]:
                    cost[followingNode] = tempCost
                    backlink[followingNode] = forwardLink
                    print(f"  Updated cost for node {followingNode}: {tempCost}")
                    print(f"  Set predecessor of node {followingNode} to link {forwardLink}")
                
                    if followingNode >= self.firstThroughNode:
                        scanList.append(followingNode)
        
        print("\nFinal shortest path results:")
        print(f"Origin: {origin}")
        for node in cost:
            if cost[node] < utils.INFINITY:
                print(f"  Node {node}: Cost = {cost[node]}, Predecessor Link = {backlink[node]}")
            else:
                print(f"  Node {node}: Unreachable")
        return (backlink, cost)

    def allOrNothing(self):
    
        print("\nStarting All-or-Nothing Assignment with Diagonalization\n" + "=" * 50)
        
        # Initialize dictionaries to hold the flow assignments for HDVs and EVs
        allOrNothing_HDV = {ij: 0 for ij in self.link}
        allOrNothing_EV = {ij: 0 for ij in self.link}

        # Loop over each origin in the network to calculate shortest paths and assign demands
        for origin in range(1, self.numZones + 1):
            print(f"\nProcessing Origin: {origin}")
            
            # Calculate shortest paths for HDVs first, then update temporary costs for EVs
            print("Calculating HDV Shortest Path")
            backlink_HDV, cost_HDV = self.shortestPath(origin, vehicleType='HDV')

            # Set temporary costs for EV calculation based on HDV path costs
            for ij in self.link:
                if self.link[ij].head in cost_HDV:
                    self.link[ij].cost_HDV = cost_HDV[self.link[ij].head]
                else:
                    self.link[ij].cost_HDV = utils.INFINITY  # For unreachable nodes

            # Now calculate shortest paths for EVs with the updated costs
            print("Calculating EV Shortest Path")
            backlink_EV, cost_EV = self.shortestPath(origin, vehicleType='EV')

            # Set temporary costs for HDV calculation based on EV path costs
            for ij in self.link:
                if self.link[ij].head in cost_EV:
                    self.link[ij].cost_EV = cost_EV[self.link[ij].head]
                else:
                    self.link[ij].cost_EV = utils.INFINITY  # For unreachable nodes

            # Now we assign the demand for each OD pair based on the shortest paths
            for OD in [OD for OD in self.ODpair if self.ODpair[OD].origin == origin]:
                destination = self.ODpair[OD].destination
                print(f"\nAssigning Demand for OD Pair: {OD} (Origin: {origin} -> Destination: {destination})")

                # Assign HDV demand to the calculated shortest path
                if backlink_HDV:
                    print("  HDV Path Assignment:")
                    curnode = destination
                    while curnode != origin:
                        link_id = backlink_HDV.get(curnode, utils.NO_PATH_EXISTS)
                        if link_id == utils.NO_PATH_EXISTS:
                            print("    No path found for HDV")
                            break
                        allOrNothing_HDV[link_id] += self.ODpair[OD].demand_HDV
                        print(f"    Link {link_id} - Added HDV Demand: {self.ODpair[OD].demand_HDV}, Total HDV Flow on Link: {allOrNothing_HDV[link_id]}")
                        curnode = self.link[link_id].tail

                # Assign EV demand to the calculated shortest path
                if backlink_EV:
                    print("  EV Path Assignment:")
                    curnode = destination
                    while curnode != origin:
                        link_id = backlink_EV.get(curnode, utils.NO_PATH_EXISTS)
                        if link_id == utils.NO_PATH_EXISTS:
                            print("    No path found for EV")
                            break
                        allOrNothing_EV[link_id] += self.ODpair[OD].demand_EV
                        print(f"    Link {link_id} - Added EV Demand: {self.ODpair[OD].demand_EV}, Total EV Flow on Link: {allOrNothing_EV[link_id]}")
                        curnode = self.link[link_id].tail

        # Print out the final assigned flows
        print("\nFinal All-or-Nothing HDV and EV Assignments with Diagonalization\n" + "=" * 50)
        for link_id in self.link:
            print(f"Link {link_id}: HDV Flow = {allOrNothing_HDV[link_id]}, EV Flow = {allOrNothing_EV[link_id]}")
        
        print("=" * 50 + "\nAll-or-Nothing Assignment with Diagonalization Complete\n")
        return allOrNothing_HDV, allOrNothing_EV

    
    def FrankWolfeStepSize(network, targetFlows_HDV, targetFlows_EV, precision=1e-7):
        #The frankWolfeStepSize function is used to determine the move sinze in the UE main function
        currentFlows_HDV = {ij: network.link[ij].flow_HDV for ij in network.link}
        currentFlows_EV = {ij: network.link[ij].flow_EV for ij in network.link}

        # Initialize bounds for binary search
        low = 0
        high = 1

        while (high - low) > precision:
            alpha = (low + high) / 2
            derivative = 0

            # Calculate the derivative of the objective function with respect to alpha
            for ij in network.link:
                # Temporarily update flows for HDVs and EVs to compute derivative
                tempFlow_HDV = alpha * targetFlows_HDV[ij] + (1 - alpha) * currentFlows_HDV[ij]
                tempFlow_EV = alpha * targetFlows_EV[ij] + (1 - alpha) * currentFlows_EV[ij]

                # Update the link flows temporarily
                network.link[ij].flow_HDV = tempFlow_HDV
                network.link[ij].flow_EV = tempFlow_EV

                # Calculate the derivative for HDVs and EVs
                derivative += ((targetFlows_HDV[ij] - currentFlows_HDV[ij]) * network.link[ij].calculateCost_HDV() +
                            (targetFlows_EV[ij] - currentFlows_EV[ij]) * network.link[ij].calculateCost_EV())
            
            # Adjust the bounds based on the sign of the derivative
            if derivative < 0:
                low = alpha
            else:
                high = alpha

        # Restore the original flows
        for ij in network.link:
            network.link[ij].flow_HDV = currentFlows_HDV[ij]
            network.link[ij].flow_EV = currentFlows_EV[ij]

        return alpha
    


    def userEquilibriumFW(self, maxIterations=100, targetGap=1e-7):
  
    # Initial all-or-nothing assignment for HDVs and EVs
        self.printCurrentLinkFlowsAndCosts()
        #Intially all the flows should be zero
        initialFlows_HDV, initialFlows_EV = self.allOrNothing()
        #Validate AON Flows should not be negative
        self.validateAON(initialFlows_HDV,initialFlows_EV)
        #Printing
        #self.printAllOrNothingAssignments(initialFlows_HDV, initialFlows_EV)
        
        for ij in self.link:
            self.link[ij].flow_HDV = initialFlows_HDV[ij]
            self.link[ij].flow_EV = initialFlows_EV[ij]
            self.link[ij].updateCosts()
        #Printing
        self.printCurrentLinkFlowsAndCosts()

        iteration = 0
        while iteration < maxIterations:
            iteration += 1

            # Compute all-or-nothing assignments for HDVs and EVs
            targetFlows_HDV, targetFlows_EV = self.allOrNothing()
            print(f"Target Flows for Iteration {iteration}")
            self.printCurrentLinkFlowsAndCosts()
            # Compute step sizes separately for HDVs and EVs
            alpha = self.FrankWolfeStepSize(targetFlows_HDV, targetFlows_EV)
            print(f"Step Size: = {alpha}")

            # Shift flows using computed step sizes for both vehicle classes
            self.shiftFlows(targetFlows_HDV, targetFlows_EV,alpha)
            self.printCurrentLinkFlowsAndCosts()


            # Convergence criteria calculation for both HDVs and EVs
            SPTT_HDV = 0
            TSTT_HDV = 0
            SPTT_EV = 0
            TSTT_EV = 0
            for ij in self.link:
                # SPTT: Shortest Path Total Travel Time
                if self.link[ij].flow_HDV < 0:
                    raise ValueError(f"Error: Negative HDV flow detected on link {ij}. Flow value: {self.link[ij].flow_HDV}")

            if self.link[ij].flow_EV < 0:
                raise ValueError(f"Error: Negative EV flow detected on link {ij}. Flow value: {self.link[ij].flow_EV}")

                    
            SPTT_HDV += self.link[ij].cost_HDV * initialFlows_HDV[ij]
            TSTT_HDV += self.link[ij].cost_HDV * self.link[ij].flow_HDV

            SPTT_EV += self.link[ij].cost_EV * initialFlows_EV[ij]
            TSTT_EV += self.link[ij].cost_EV * self.link[ij].flow_EV

            # Compute gap for each vehicle type and combined gap
            gap_HDV = TSTT_HDV / SPTT_HDV - 1 if SPTT_HDV != 0 else 0
            gap_EV = TSTT_EV / SPTT_EV - 1 if SPTT_EV != 0 else 0
            combined_gap = max(gap_HDV, gap_EV)

            print(f"SPTT_HDV: {SPTT_HDV}, TSTT_HDV: {TSTT_HDV}, gap_HDV: {gap_HDV}")
            print(f"SPTT_EV: {SPTT_EV}, TSTT_EV: {TSTT_EV}, gap_EV: {gap_EV}")
            print(f"Maximumn Gap: {combined_gap}")

            print("Iteration %d: gap HDV %f, gap EV %f, combined gap %f, Beckmann value %f" %
                (iteration, gap_HDV, gap_EV, combined_gap, self.beckmannFunction()))

            # Check convergence using the combined gap
            if combined_gap < targetGap:
                break

        

    def readFromFiles(self, networkFile, demandFile):
        """
        Reads network data from a pair of files (networkFile, containing the topology,
        and demandFile, containing the OD matrix) and do some basic checks on
        the input data (validate)"""
        self.readNetworkFile(networkFile)
        self.readDemandFile(demandFile)
        self.validate()
        self.finalize()


    def readNetworkFile(self, networkFileName):
        """
        Reads network topology data from the TNTP data format.  In keeping with
        this format, the zones/centroids are assumed to have the lowest node
        IDs (1, 2, ..., numZones).
        """
        try:
            with open(networkFileName, "r") as networkFile:
                fileLines = networkFile.read().splitlines()

                # Set default parameters for metadata, then read
                self.numNodes = None
                self.numLinks = None
                self.numZones = None
                self.firstThroughNode = 0
                metadata = utils.readMetadata(fileLines)

                try:
                    self.numNodes = int(metadata['NUMBER OF NODES'])
                    self.numLinks = int(metadata['NUMBER OF LINKS'])
                    if self.numZones != None:
                        if self.numZones != int(metadata['NUMBER OF ZONES']):
                            print("Error: Number of zones does not match in network/demand files.")
                            raise utils.BadFileFormatException
                    else:
                        self.numZones = int(metadata['NUMBER OF ZONES'])
                    self.firstThroughNode = int(metadata['FIRST THRU NODE'])
                except KeyError: # KeyError
                    print("Warning: Not all metadata present, error checking will be limited and code will proceed as though all nodes are through nodes.")
                self.tollFactor = float(metadata.setdefault('TOLL FACTOR', 0))
                self.distanceFactor = float(metadata.setdefault('DISTANCE FACTOR', 0))

                for line in fileLines[metadata['END OF METADATA']:]:
                    # Ignore comments and blank lines
                    line = line.strip()
                    commentPos = line.find("~")
                    if commentPos >= 0: # strip comments
                        line = line[:commentPos]

                    if len(line) == 0:
                        continue

                    data = line.split()
                    if len(data) < 11 or data[10] != ';' :
                        print("Link data line not formatted properly:\n '%s'" % line)
                        raise utils.BadFileFormatException

                    # Create link
                    linkID = '(' + str(data[0]).strip() + "," + str(data[1]).strip() + ')'

                    self.link[linkID] = Link(self,
                            int(data[0]), int(data[1]), # head and tail
                            float(data[2]),   # capacity
                            float(data[3]),   # length
                            float(data[4]),   # free-flow time
                            float(data[5]),   # BPR alpha
                            float(data[6]),   # BPR beta
                            float(data[7]),   # Speed limit
                            float(data[8]),   # Toll
                            data[9])          # Link type

                    # Create nodes if necessary
                    if data[0] not in self.node: # tail
                        self.node[int(data[0])] = Node(True if int(data[0]) <= self.numZones else False)
                    if data[1] not in self.node: # head
                        self.node[int(data[1])] = Node(True if int(data[1]) <= self.numZones else False)

        except IOError:
            print("\nError reading network file %s" % networkFile)
            traceback.print_exc(file=sys.stdout)

    def readDemandFile(self, demandFileName):
        """
        Reads demand (OD matrix) data from a file in the TNTP format.
        """
        try:
            with open(demandFileName, "r") as demandFile:
                fileLines = demandFile.read().splitlines()
                self.totalDemand = 0

                # Set default parameters for metadata, then read
                self.totalDemandCheck = None

                metadata = utils.readMetadata(fileLines)
                try:
                    self.totalDemandCheck = float(metadata['TOTAL OD FLOW'])
                    if self.numZones != None:
                        if self.numZones != int(metadata['NUMBER OF ZONES']):
                            print("Error: Number of zones does not match in network/demand files.")
                            raise utils.BadFileFormatException
                    else:
                        self.numZones = int(metadata['NUMBER OF ZONES'])

                except KeyError: # KeyError
                    print("Warning: Not all metadata present in demand file, error checking will be limited.")

                for line in fileLines[metadata['END OF METADATA']:]:
                    # Ignore comments and blank lines
                    line = line.strip()
                    commentPos = line.find("~")
                    if commentPos >= 0: # strip comments
                        line = line[:commentPos]
                    if len(line) == 0:
                        continue

                    data = line.split()

                    if data[0] == 'Origin':
                        origin = int(data[1])
                        continue

                    # Two possibilities, either semicolons are directly after values or there is an intervening space
                    if len(data) % 3 != 0 and len(data) % 4 != 0:
                        print("Demand data line not formatted properly:\n %s" % line)
                        raise utils.BadFileFormatException

                    for i in range(int(len(data) // 3)):
                        destination = int(data[i * 3])
                        check = data[i * 3 + 1]
                        demand = data[i * 3 + 2]
                        demand = float(demand[:len(demand)-1])
                        if check != ':' :
                            print("Demand data line not formatted properly:\n %s" % line)
                            raise utils.BadFileFormatException
                        ODID = str(origin) + '->' + str(destination)
                        percentage_EV = 0
                        percentage_HDV = 1
                        self.ODpair[ODID] = OD(origin, destination, demand,percentage_HDV,percentage_EV)
                        self.totalDemand += demand

        except IOError:
            print("\nError demand  file %s" % demandFile)
            traceback.print_exc(file=sys.stdout)

    def validate(self):
        """
        Perform some basic validation checking of network, link, and node
        data to ensure reasonableness and consistency.
        """
        valid = True

        # Check that link information is valid
        for ij in self.link:
            valid = valid and self.link[ij].head in self.node
            valid = valid and self.link[ij].tail in self.node
            if not valid:
                print("Error: Link tail/head not found: %s %s" % (self.link[ij].tail, self.link[ij].head))
                raise utils.BadFileFormatException
            valid = valid and self.link[ij].capacity >= 0
            valid = valid and self.link[ij].length >= 0
            valid = valid and self.link[ij].freeFlowTime >= 0
            valid = valid and self.link[ij].alpha >= 0
            valid = valid and self.link[ij].beta >= 0
            valid = valid and self.link[ij].speedLimit >= 0
            valid = valid and self.link[ij].toll >= 0
            if not valid:
                print("Link %s has negative parameters." % ij)

        # Then check that all OD pairs are in range
        for ODpair in self.ODpair:
            (origin, destination) = (self.ODpair[ODpair].origin, self.ODpair[ODpair].destination)
            valid = valid and origin in self.node
            valid = valid and destination in self.node
            if not valid:
                print("Error: Origin/destination %s not found" % ODpair)
                raise utils.BadFileFormatException
            valid = valid and self.node[origin].isZone == True
            valid = valid and self.node[destination].isZone == True
            if not valid:
                print("Error: Origin/destination %s does not connect two zones" % str(ODpair))
                raise utils.BadFileFormatException
            valid = valid and self.ODpair[ODpair].demand >= 0
            if not valid:
                print("Error: OD pair %s has negative demand" % ODpair)
                raise utils.BadFileFormatException

        # Now error-check using metadata
        if self.numNodes != None and len(self.node) != self.numNodes:
            print("Warning: Number of nodes implied by network file %d different than metadata value %d" % (len(self.node), self.numNodes))
            self.numNodes = len(self.node)
        if self.numLinks != None and len(self.link) != self.numLinks:
            print("Warning: Number of links given in network file %d different than metadata value %d" % (len(self.link), self.numLinks))
            self.numLinks = len(self.link)
        if self.numZones != None and len([i for i in self.node if self.node[i].isZone == True]) != self.numZones:
            print("Warning: Number of zones given in network file %d different than metadata value %d" % (len([i for i in self.node if self.node[i].isZone == True]), self.numZones))
            self.numLinks = len(self.link)
        if self.totalDemandCheck != None:
            if self.totalDemand != self.totalDemandCheck:
                print("Warning: Total demand is %f compared to metadata value %f" % ( self.totalDemand, self.totalDemandCheck))

    def finalize(self):
        """
        Establish the leavingLinks and enteringLinks lists for nodes, initialize flows and
        costs for links and OD pairs.
        """
        # Establish forward/reverse star lists, set travel times to free-flow
        for i in self.node:
            self.node[i].leavingLinks = list()
            self.node[i].enteringLinks = list()
            
        for ij in self.link:
            self.node[self.link[ij].tail].leavingLinks.append(ij)
            self.node[self.link[ij].head].enteringLinks.append(ij)
            self.link[ij].cost = self.link[ij].freeFlowTime + self.link[ij].length * self.distanceFactor + self.link[ij].toll * self.tollFactor
            self.link[ij].flow = 0
            
        for OD in self.ODpair:
            self.ODpair[OD].leastCost = 0

    #Additional functions to print validations and intial data sets
    def printLinkData(self):
    
        print("Network Link Data:")
        print("="*50)
    
        for link_id, link in self.link.items():
            print(f"Link ID: {link_id}")
            print(f"  - Tail Node: {link.tail}")
            print(f"  - Head Node: {link.head}")
            print(f"  - Orignal Capacity: {link.capacity}")
            print(f"  - Capacity (HDV): {link.capacity_HDV}")
            print(f"  - Capacity (EV): {link.capacity_EV}")
            print(f"  - Length: {link.length}")
            print(f"  - Free Flow Time: {link.freeFlowTime}")
            print(f"  - Free Flow Speed: {link.freeFlowSpeed}")
            print("="*50)

    def printODData(self):
   
        print("=" * 50)
        print("Origin-Destination Pairs Data")
        print("=" * 50)
        
        for od_id, od_pair in self.ODpair.items():
            print(f"OD ID: {od_id}")
            print(f"Origin: {od_pair.origin}")
            print(f"Destination: {od_pair.destination}")
            print(f"HDV Demand: {od_pair.demand_HDV}")
            print(f"EV Demand: {od_pair.demand_EV}")
            print("=" * 50)



    def export_link_data_to_csv(self, filename="link_data.csv"):
        with open(filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            # Write the header row
            writer.writerow(["Link ID", "Tail Node", "Head Node", "Original Capacity", 
                            "Capacity (HDV)", "Capacity (EV)", "Length", 
                            "Free Flow Time", "Free Flow Speed"])
            
            # Write each link's data
            for link_id, link in self.link.items():
                writer.writerow([
                    link_id,
                    link.tail,
                    link.head,
                    link.capacity,
                    link.capacity_HDV,
                    link.capacity_EV,
                    link.length,
                    link.freeFlowTime,
                    link.freeFlowSpeed
                ])
        print(f"Link data exported to {filename}")

    def export_od_data_to_csv(self, filename="od_data.csv"):
        """
        Exports OD pair data to a CSV file.
        """
        try:
            with open(filename, mode="w", newline="") as file:
                writer = csv.writer(file)
                # Write the header row
                writer.writerow(["OD ID", "Origin", "Destination", "HDV Demand", "EV Demand"])
                
                # Write each OD pair's data
                for od_id, od_pair in self.ODpair.items():
                    writer.writerow([
                        od_id,
                        od_pair.origin,
                        od_pair.destination,
                        od_pair.demand_HDV,
                        od_pair.demand_EV
                    ])
            print(f"OD pair data successfully exported to {filename}")

        except AttributeError as e:
            print("AttributeError encountered during CSV export of OD data:", e)
        except Exception as e:
            print("An error occurred during CSV export of OD data:", e)

    def printAllOrNothingAssignments(self, allOrNothing_HDV, allOrNothing_EV):
    
        print("All-or-Nothing Assignment Results")
        print("=" * 50)
        
        for link_id in self.link:
            hdv_flow = allOrNothing_HDV.get(link_id, 0)
            ev_flow = allOrNothing_EV.get(link_id, 0)
            
            print(f"Link ID: {link_id}")
            print(f"  - HDV Flow: {hdv_flow}")
            print(f"  - EV Flow: {ev_flow}")
            print("=" * 50)


    def printCurrentLinkFlowsAndCosts(self):
   
        print("Current Link Flows and Costs")
        print("=" * 50)
        
        for link_id, link in self.link.items():
            print(f"Link ID: {link_id}")
            print(f"  - HDV Flow: {link.flow_HDV}")
            print(f"  - EV Flow: {link.flow_EV}")
            print(f"  - HDV Cost: {link.cost_HDV}")
            print(f"  - EV Cost: {link.cost_EV}")
            print("=" * 50)

    def validateAON(self,allOrNothing_HDV, allOrNothing_EV):
        for link_id, flow in allOrNothing_HDV.items():
            if flow < 0:
                raise ValueError(f"Error: Negative flow detected for HDV on link {link_id}.")
        for link_id, flow in allOrNothing_EV.items():
            if flow < 0:
                raise ValueError(f"Error: Negative flow detected for EV on link {link_id}.")
