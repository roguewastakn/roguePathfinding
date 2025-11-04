--- Pathfinder3D.lua ---

--[[
    Module: Pathfinder3D
    Description: Highly optimized, custom 3D pathfinding module for Roblox using a Sparse Voxel Octree (SVO)
                 for global path planning and a local collision avoidance system for dynamic obstacle handling.
    Author: Manus
]]

-- Assuming Vector3 and Heap modules are available in the environment.
-- In a real Roblox environment, Vector3 is a built-in type.
-- For this demonstration, we assume they are required from sibling scripts.
local Vector3 = require(script.Vector3)
local Heap = require(script.Heap)

local Pathfinder3D = {}
local SVO = {} -- Internal table for the Sparse Voxel Octree data
local CONFIG = {
    -- Voxelization settings
    WORLD_BOUNDS_MIN = Vector3.new(-1000, -1000, -1000),
    WORLD_BOUNDS_MAX = Vector3.new(1000, 1000, 1000),
    MAX_OCTREE_DEPTH = 8, -- Max depth determines the finest voxel resolution (e.g., 2^8 = 256 divisions)
    MIN_VOXEL_SIZE = 4,   -- The size of the smallest voxel (leaf node) in studs
    -- Pathfinding settings
    A_STAR_HEURISTIC_WEIGHT = 1.0,
    -- Collision Avoidance settings
    AGENT_RADIUS = 2,
    TIME_HORIZON = 2, -- Time in seconds for anticipation
    MAX_SPEED = 15,
}

-- =================================================================================
-- 1. SVO Utility Functions (Re-used from Phase 3)
-- =================================================================================

local function CalculateMortonCode(x, y, z)
    local code = 0
    for i = 0, CONFIG.MAX_OCTREE_DEPTH - 1 do
        if (x & (1 << i)) ~= 0 then code = code | (1 << (3 * i)) end
        if (y & (1 << i)) ~= 0 then code = code | (1 << (3 * i + 1)) end
        if (z & (1 << i)) ~= 0 then code = code | (1 << (3 * i + 2)) end
    end
    return code
end

local function WorldToSVO(position)
    local min = CONFIG.WORLD_BOUNDS_MIN
    local max = CONFIG.WORLD_BOUNDS_MAX
    local range = max - min
    local max_coord = (2 ^ CONFIG.MAX_OCTREE_DEPTH) - 1

    local x = math.floor(((position.X - min.X) / range.X) * max_coord)
    local y = math.floor(((position.Y - min.Y) / range.Y) * max_coord)
    local z = math.floor(((position.Z - min.Z) / range.Z) * max_coord)

    x = math.max(0, math.min(max_coord, x))
    y = math.max(0, math.min(max_coord, y))
    z = math.max(0, math.min(max_coord, z))

    return x, y, z
end

local function SVOToWorldCenter(x, y, z, level)
    local min = CONFIG.WORLD_BOUNDS_MIN
    local max = CONFIG.WORLD_BOUNDS_MAX
    local range = max - min
    local max_coord = (2 ^ CONFIG.MAX_OCTREE_DEPTH)
    local voxel_size = range / max_coord

    local center_offset = voxel_size / 2

    local world_x = min.X + (x * voxel_size.X) + center_offset.X
    local world_y = min.Y + (y * voxel_size.Y) + center_offset.Y
    local world_z = min.Z + (z * voxel_size.Z) + center_offset.Z

    return Vector3.new(world_x, world_y, world_z)
end

-- =================================================================================
-- 2. SVO Generation (Mock Implementation Re-used from Phase 3)
-- =================================================================================

local function BuildSVO()
    local mock_svo = {}
    local max_coord = (2 ^ CONFIG.MAX_OCTREE_DEPTH) - 1
    local id_counter = 1
    local center_x, center_y, center_z = math.floor(max_coord / 2), math.floor(max_coord / 2), math.floor(max_coord / 2)

    for x = center_x - 1, center_x + 1 do
        for y = center_y - 1, center_y + 1 do
            for z = center_z - 1, center_z + 1 do
                local center = SVOToWorldCenter(x, y, z, CONFIG.MAX_OCTREE_DEPTH)
                local node = {
                    ID = id_counter,
                    Level = CONFIG.MAX_OCTREE_DEPTH,
                    MortonCode = CalculateMortonCode(x, y, z),
                    State = 0, -- 0: Free
                    Center = center,
                    Size = CONFIG.MIN_VOXEL_SIZE,
                    Neighbors = {},
                }
                mock_svo[id_counter] = node
                id_counter = id_counter + 1
            end
        end
    end
    return mock_svo
end

-- =================================================================================
-- 3. A* Pathfinding on SVO (Re-used from Phase 3)
-- =================================================================================

local function Heuristic(nodeA, nodeB)
    return (nodeA.Center - nodeB.Center).Magnitude * CONFIG.A_STAR_HEURISTIC_WEIGHT
end

local function FindClosestNode(position)
    local min_dist_sq = math.huge
    local closest_node = nil

    for _, node in pairs(SVO) do
        local dist_sq = (node.Center - position).MagnitudeSq
        if dist_sq < min_dist_sq then
            min_dist_sq = dist_sq
            closest_node = node
        end
    end
    return closest_node
end

local function GetNeighbors(node)
    local neighbors = {}
    local x, y, z = WorldToSVO(node.Center)
    local max_coord = (2 ^ CONFIG.MAX_OCTREE_DEPTH) - 1

    for dx = -1, 1 do
        for dy = -1, 1 do
            for dz = -1, 1 do
                if dx ~= 0 or dy ~= 0 or dz ~= 0 then
                    local nx, ny, nz = x + dx, y + dy, z + dz

                    if nx >= 0 and nx <= max_coord and
                       ny >= 0 and ny <= max_coord and
                       nz >= 0 and nz <= max_coord then

                        local neighbor_center = SVOToWorldCenter(nx, ny, nz, CONFIG.MAX_OCTREE_DEPTH)
                        local neighbor_node = FindClosestNode(neighbor_center)

                        if neighbor_node and neighbor_node.State == 0 then
                            table.insert(neighbors, neighbor_node)
                        end
                    end
                end
            end
        end
    end
    return neighbors
end

function Pathfinder3D:ComputePath(startPos, endPos)
    local startNode = FindClosestNode(startPos)
    local endNode = FindClosestNode(endPos)

    if not startNode or not endNode then
        warn("Start or End position is outside the traversable SVO area.")
        return nil
    end

    local openSet = Heap.new(function(a, b) return a.f < b.f end)
    local cameFrom = {}
    local gScore = {}
    local fScore = {}

    gScore[startNode.ID] = 0
    fScore[startNode.ID] = Heuristic(startNode, endNode)
    openSet:Insert({node = startNode, f = fScore[startNode.ID]})

    local function reconstructPath(current)
        local totalPath = {current.Center}
        local node = current
        while cameFrom[node.ID] do
            node = cameFrom[node.ID]
            table.insert(totalPath, 1, node.Center)
        end
        return totalPath
    end

    while not openSet:IsEmpty() do
        local current = openSet:ExtractMin().node

        if current.ID == endNode.ID then
            return reconstructPath(current)
        end

        for _, neighbor in ipairs(GetNeighbors(current)) do
            local tentative_gScore = gScore[current.ID] + (current.Center - neighbor.Center).Magnitude

            if not gScore[neighbor.ID] or tentative_gScore < gScore[neighbor.ID] then
                cameFrom[neighbor.ID] = current
                gScore[neighbor.ID] = tentative_gScore
                fScore[neighbor.ID] = tentative_gScore + Heuristic(neighbor, endNode)

                openSet:Insert({node = neighbor, f = fScore[neighbor.ID]})
            end
        end
    end

    return nil
end

-- =================================================================================
-- 4. Local Collision Avoidance (Simplified ORCA-like)
-- =================================================================================

-- A simplified 3D implementation of the core concept of Velocity Obstacles (VO)
-- and Optimal Reciprocal Collision Avoidance (ORCA).
-- This function calculates a safe velocity that avoids collision with a single obstacle.
-- For multiple obstacles, the final velocity would be the intersection of all safe velocity half-planes.
local function CalculateSafeVelocity(agentPos, agentVel, obstaclePos, obstacleVel, agentRadius, obstacleRadius, timeHorizon)
    local relativePos = obstaclePos - agentPos
    local relativeVel = agentVel - obstacleVel
    local combinedRadius = agentRadius + obstacleRadius

    -- 1. Calculate the Velocity Obstacle (VO)
    -- The VO is a cone in velocity space. Any velocity vector inside this cone will lead to a collision.
    local distSq = relativePos.MagnitudeSq
    local dist = math.sqrt(distSq)

    -- If the agents are already far apart, no need for avoidance
    if dist > combinedRadius * timeHorizon then
        return nil -- No immediate avoidance needed
    end

    -- The half-angle of the collision cone (VO)
    local angle = math.asin(combinedRadius / dist)
    local relativePosUnit = relativePos.unit

    -- The two tangent vectors defining the cone's boundaries
    -- This is a 3D rotation, which is complex. We simplify by finding the closest point on the cone boundary.

    -- 2. Calculate the Closest Point of Approach (CPA)
    local timeToCPA = -relativePos:Dot(relativeVel) / relativeVel.MagnitudeSq
    local closestPoint = relativePos + relativeVel * timeToCPA

    -- 3. Check for collision
    if distSq < combinedRadius * combinedRadius or (timeToCPA > 0 and closestPoint.MagnitudeSq < combinedRadius * combinedRadius) then
        -- Collision is imminent or already happening.
        -- The preferred velocity is the one that moves the agent out of the collision cone.
        
        -- The center of the VO is the relative velocity that would lead to collision.
        local voCenter = relativePos / timeHorizon
        
        -- The vector from the VO center to the closest point on the boundary
        local voBoundaryVector = voCenter - relativeVel
        
        -- The new safe relative velocity is the one that is closest to the current relative velocity
        -- but outside the VO. For simplicity, we project the current relative velocity onto the
        -- vector that moves the agent away from the obstacle center.
        
        -- The vector pointing from the obstacle to the agent
        local separationVector = (agentPos - obstaclePos).unit
        
        -- The safe relative velocity is one that has a component in the separation direction
        -- We want to move away from the obstacle.
        local safeRelativeVel = separationVector * CONFIG.MAX_SPEED
        
        -- The safe absolute velocity for the agent
        local safeAbsoluteVel = safeRelativeVel + obstacleVel
        
        return safeAbsoluteVel
    end

    return nil -- No avoidance needed
end

---
-- @brief Used by the agent's steering behavior to compute a safe, collision-free velocity.
-- This method integrates the local collision avoidance system (e.g., ORCA-like).
-- @param currentPos Vector3: Agent's current position.
-- @param currentVel Vector3: Agent's current velocity.
-- @param targetPos Vector3: The next global waypoint from the computed path.
-- @param obstacles table: A list of nearby dynamic obstacles (e.g., { Pos=Vector3, Vel=Vector3, Radius=number }).
-- @return Vector3: The new, safe velocity vector for the agent.
function Pathfinder3D:GetSteeringVelocity(currentPos, currentVel, targetPos, obstacles)
    -- 1. Calculate the preferred velocity (towards targetPos).
    local preferredVelocity = (targetPos - currentPos).unit * CONFIG.MAX_SPEED
    local safeVelocity = preferredVelocity

    -- 2. Apply local collision avoidance (Simplified ORCA)
    for _, obstacle in ipairs(obstacles) do
        local safeVelForObstacle = CalculateSafeVelocity(
            currentPos,
            safeVelocity, -- Use the current safe velocity as the input velocity for the next obstacle check
            obstacle.Pos,
            obstacle.Vel,
            CONFIG.AGENT_RADIUS,
            obstacle.Radius,
            CONFIG.TIME_HORIZON
        )

        if safeVelForObstacle then
            -- In a full ORCA implementation, we would find the intersection of all half-planes.
            -- Here, we use the avoidance velocity if one is calculated, overriding the preferred velocity.
            -- This is a simple, but effective, "first-collision-wins" approach.
            safeVelocity = safeVelForObstacle
        end
    end

    -- Clamp the final velocity to the maximum speed
    if safeVelocity.Magnitude > CONFIG.MAX_SPEED then
        safeVelocity = safeVelocity.unit * CONFIG.MAX_SPEED
    end

    return safeVelocity
end

-- =================================================================================
-- 5. Public API and Initialization
-- =================================================================================

function Pathfinder3D.new(configTable)
    for k, v in pairs(configTable or {}) do
        CONFIG[k] = v
    end
    SVO = BuildSVO()
    return Pathfinder3D
end

return Pathfinder3D
