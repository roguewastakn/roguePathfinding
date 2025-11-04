--- Heap.lua (Min-Heap implementation for A*) ---

local Heap = {}
Heap.__index = Heap

function Heap.new(compareFunc)
    local self = setmetatable({}, Heap)
    self.data = {}
    self.compare = compareFunc or function(a, b) return a < b end -- Default to Min-Heap
    return self
end

function Heap:IsEmpty()
    return #self.data == 0
end

function Heap:Insert(value)
    table.insert(self.data, value)
    self:BubbleUp(#self.data)
end

function Heap:ExtractMin()
    if self:IsEmpty() then return nil end
    if #self.data == 1 then return table.remove(self.data) end

    local min = self.data[1]
    self.data[1] = table.remove(self.data)
    self:BubbleDown(1)
    return min
end

function Heap:BubbleUp(index)
    local parentIndex = math.floor(index / 2)
    while index > 1 and self.compare(self.data[index], self.data[parentIndex]) do
        self.data[index], self.data[parentIndex] = self.data[parentIndex], self.data[index]
        index = parentIndex
        parentIndex = math.floor(index / 2)
    end
end

function Heap:BubbleDown(index)
    local leftChildIndex = index * 2
    local rightChildIndex = index * 2 + 1
    local smallest = index

    if leftChildIndex <= #self.data and self.compare(self.data[leftChildIndex], self.data[smallest]) then
        smallest = leftChildIndex
    end

    if rightChildIndex <= #self.data and self.compare(self.data[rightChildIndex], self.data[smallest]) then
        smallest = rightChildIndex
    end

    if smallest ~= index then
        self.data[index], self.data[smallest] = self.data[smallest], self.data[index]
        self:BubbleDown(smallest)
    end
end

return Heap
