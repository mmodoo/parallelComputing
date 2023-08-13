#include <bits/stdc++.h>

#include <algorithm>
#include <random>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <numeric>
#include <string>

constexpr int INFINITE = std::numeric_limits<int>::max();

using namespace std;

typedef pair<int, int> dot;
typedef pair<int, dot> weight;

#define ROW MAP_SIZE
#define COL MAP_SIZE

/**
 *  change simulation settings
 *  TIME_MAX : total time steps
 *	map generated will be MAP_SIZE x MAP_SIZE grid
 *	NUM_ROBOTS : number of robots in simulation
 *  NUM_RTYPE : number of robot types (currently DRONE, CATERPILLAR , WHEEL
 *  NUM_MAX_TASKS : total number of tasks to be done
 *  NUM_INITIAL_TASKS : number of tasks generated at the beginning of simulation
 *	MAX_ENERGY : total energy that a single robot has at the beginning of simulation
 *  WALL_DENSITY : probability of a grid coordinate being a wall
 *  ENERGY_CONSUMPTION_PER_TICK : energy consumed every timestep when working, or moving
 *
 *  SEED : random seed. with same seed, simulator will generate exactly same random results including (map, object, tasks, actions etc.)
 *  SIMULATOR_VERBOSE : if true, print out maps
 */
constexpr int MAP_SIZE = 40;
constexpr int TIME_MAX = MAP_SIZE * 100;
constexpr int NUM_ROBOT = 6;
constexpr int NUM_RTYPE = 3;
constexpr int NUM_MAX_TASKS = 16;
constexpr int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
constexpr int MAX_ENERGY = TIME_MAX * 6;
constexpr int WALL_DENSITY = 10;
constexpr int ENERGY_CONSUMPTION_PER_TICK = 10;
constexpr int TASK_PROGRESS_PER_TICK = ENERGY_CONSUMPTION_PER_TICK;

constexpr unsigned int SEED = 1;
constexpr bool SIMULATOR_VERBOSE = true;

#pragma region CONSTANT

enum ObjectType : int
{
	UNKNOWN = -1,
	EMPTY = 0b0000,
	ROBOT = 0b0001,
	TASK = 0b0010,
	ROBOT_AND_TASK = 0b0011,
	WALL = 0b0100
};
ObjectType operator&(ObjectType lhs, ObjectType rhs) { return static_cast<ObjectType>(static_cast<int>(lhs) & static_cast<int>(rhs)); }
ObjectType operator|(ObjectType lhs, ObjectType rhs) { return static_cast<ObjectType>(static_cast<int>(lhs) | static_cast<int>(rhs)); }
ObjectType &operator|=(ObjectType &lhs, ObjectType rhs) { return lhs = lhs | rhs; }
ObjectType &operator&=(ObjectType &lhs, ObjectType rhs) { return lhs = lhs & rhs; }
ObjectType &operator&=(ObjectType &lhs, unsigned long rhs) { return lhs = static_cast<ObjectType>(static_cast<int>(lhs) & rhs); }

enum RobotType
{
	INVALID = -1,
	DRONE,
	CATERPILLAR,
	WHEEL,
	OBJECT,
	NUMBER
};
constexpr RobotType robot_types[3] = {RobotType::DRONE, RobotType::CATERPILLAR, RobotType::WHEEL};
std::string robot_names[3] = {"DRONE", "CATERPILLAR", "WHEEL"};

int terreinMatrix[NUM_RTYPE][MAP_SIZE][MAP_SIZE];
int objectMatrix[MAP_SIZE][MAP_SIZE];
int numRobotMatrix[MAP_SIZE][MAP_SIZE];

int knownTerrein[NUM_RTYPE][MAP_SIZE][MAP_SIZE];
int knownObject[MAP_SIZE][MAP_SIZE];

int scoreF[101][101];
int scoreG[101][101];
int scoreH[101][101];

bool distance_calculate = false;

bool DRONE1_is_ready = false;
bool DRONE1_step1 = false;
bool DRONE1_step2 = false;
bool DRONE1_step3 = false;

bool DRONE2_is_ready = false;
bool DRONE2_step1 = false;
bool DRONE2_step2 = false;
bool DRONE2_step3 = false;

int x1 = 1;
int x2 = 1;
int x3 = 1;
int x4 = 1;

int ii = 0;

int energy_constant;

class Coord
{
public:
	int x;
	int y;

	constexpr Coord() : x{-1}, y{-1} {}
	constexpr Coord(int xx, int yy) : x{xx}, y{yy} {}

	friend std::ostream &operator<<(std::ostream &o, const Coord &coord)
	{
		o << "{" << coord.x << ", " << coord.y << "}";
		return o;
	}
};

Coord operator+(const Coord &lhs, const Coord &rhs)
{
	return {lhs.x + rhs.x, lhs.y + rhs.y};
}

Coord operator-(const Coord &lhs, const Coord &rhs)
{
	return {lhs.x - rhs.x, lhs.y - rhs.y};
}

bool operator==(const Coord &lhs, const Coord &rhs)
{
	return lhs.x == rhs.x && rhs.y == lhs.y;
}

bool operator!=(const Coord &lhs, const Coord &rhs)
{
	return !(lhs == rhs);
}

namespace std
{
	template <>
	struct hash<Coord>
	{
		std::size_t operator()(const Coord &c) const noexcept
		{
			unsigned long long hash = c.x << 4;
			hash |= c.y;
			return hash;
		}
	};
} // namespace std

class Robot;
class Task;
class TaskView;

/**
 * @brief Returns the reference of the object at a specified position.
 *
 * @param coord The position.
 * @return ObjectType& The reference of the object.
 */
ObjectType &object_at(Coord coord)
{
	int *ptr = *(objectMatrix + coord.x) + coord.y;
	ObjectType *casted = reinterpret_cast<ObjectType *>(ptr);

	return *casted;
}

ObjectType &known_object_at(Coord coord)
{
	int *ptr = *(knownObject + coord.x) + coord.y;
	ObjectType *casted = reinterpret_cast<ObjectType *>(ptr);

	return *casted;
}

int &terrein_cost_at(RobotType type, Coord coord) { return *(*(*(terreinMatrix + static_cast<int>(type)) + coord.x) + coord.y); }

int &known_terrein_cost_at(RobotType type, Coord coord) { return *(*(*(knownTerrein + static_cast<int>(type)) + coord.x) + coord.y); }

int &num_robots_at(Coord coord) { return *(*(numRobotMatrix + coord.x) + coord.y); }

Coord get_random_empty_position();

void reveal_square_range(Coord centre, int view_range, std::unordered_map<Coord, Task *> &uncharted_tasks, std::vector<TaskView> &active_tasks);

void reveal_cross_range(Coord center, int view_range, std::unordered_map<Coord, Task *> &uncharted_tasks, std::vector<TaskView> &active_tasks);

std::chrono::high_resolution_clock::time_point get_now() { return std::chrono::high_resolution_clock::now(); }

void generateMap(Robot *robots, Task *all_tasks, std::unordered_map<Coord, Task *> &uncharted_tasks, std::vector<TaskView> &active_tasks);

void printMap(int);

void printObjects(const Robot *robots, const Task *all_tasks, int num_tasks);

static const int task_num_length = std::to_string(NUM_MAX_TASKS).length();

#pragma endregion CONSTANT

class Task
{
	static constexpr int DRONE_DEFAULT_COST = MAX_ENERGY;
	static constexpr int CATERPILLAR_COST_EXCLUSIVE_UPPER_BOUND = 100;
	static constexpr int CATERPILLAR_COST_MIN = 50;
	static constexpr int WHEEL_COST_EXCLUSIVE_UPPER_BOUND = 200;
	static constexpr int WHEEL_COST_MIN = 0;

public:
	Coord coord;
	int id;
	bool done;
	int assigned_robot_id;

	/**
	 * @brief The default constructor that constructs an invalid task.
	 */
	Task()
		: id{-1}, done{false}
	{
		// std::fill(taskCost_, taskCost_ + NUM_RTYPE, INFINITE);
	}

	Task(Coord position, int id, std::initializer_list<int> costs)
		: coord{position}, id{id}, done{false}
	{
		assert(costs.size() == NUM_RTYPE);
		std::copy(costs.begin(), costs.end(), taskCost_);
	}

	int get_cost_by_type(RobotType type) const
	{
		assert(("Requires a valid robot type.", type != RobotType::INVALID));
		return taskCost_[type];
	}

	static Task generate_random(int id)
	{
		Coord position = get_random_empty_position();

		return Task(position, id,
					// Costs
					{
						DRONE_DEFAULT_COST,
						rand() % CATERPILLAR_COST_EXCLUSIVE_UPPER_BOUND + CATERPILLAR_COST_MIN,
						rand() % WHEEL_COST_EXCLUSIVE_UPPER_BOUND + WHEEL_COST_MIN});
	}

	friend std::ostream &operator<<(std::ostream &o, const Task &task)
	{
		o << "Task[" << std::setw(task_num_length) << task.id << "]: ";
		o << "Location " << task.coord << "\t";
		if (task.done)
			o << "Done (by Robot " << task.assigned_robot_id << ")";
		else
			o << "Not done";
		return o;
	}

private:
	int taskCost_[NUM_RTYPE];
};

class TaskView
{
public:
	int id() const noexcept { return task_->id; }
	/**
	 * @brief Returns the true position of this task.
	 */
	Coord coord() const noexcept { return task_->coord; }
	/**
	 * @brief Get the cost by a specified robot type.
	 */
	int get_cost_by_type(RobotType type) const noexcept { return task_->get_cost_by_type(type); }

	TaskView(const Task &task) : task_{std::addressof(task)} {}
	friend void reveal_square_range(Coord centre, int view_range, std::vector<TaskView> &active_tasks);
	friend void reveal_cross_range(Coord center, int view_range, std::vector<TaskView> &active_tasks);

private:
	const Task *task_;
};

struct DispatchResult
{
	bool success;
	Task task;
};

class TaskDispatcher
{
public:
	TaskDispatcher()
		: next_task_id_{NUM_INITIAL_TASKS}, next_task_arrival_time_{TIME_MAX / 4}
	{
	}

	DispatchResult try_dispatch(int current_time)
	{
		if (current_time == next_task_arrival_time_ && next_task_id_ < NUM_MAX_TASKS)
		{
			if (SIMULATOR_VERBOSE)
				std::cout << "At time " << time << ": ";

			next_task_arrival_time_ += (TIME_MAX / 2) / (NUM_MAX_TASKS / 2);

			Task new_task{Task::generate_random(next_task_id_)};

			next_task_id_++;

			return {true, new_task};
		}
		else
		{
			return {false, {}};
		}
	}

	int next_task_id() const noexcept { return next_task_id_; }
	int num_remaining_tasks() const noexcept { return NUM_MAX_TASKS - next_task_id_; }

private:
	int next_task_id_;
	int next_task_arrival_time_;
};

enum Status
{
	IDLE,
	WORKING,
	MOVING,
	EXHAUSTED
};
static const std::string status_strs[] = {"IDLE", "WORKING", "MOVING", "EXHAUSTED"};

enum Action
{
	UP,
	DOWN,
	LEFT,
	RIGHT,
	HOLD
};

class Robot
{
public:
	/**
	 * @brief Robot ID
	 */
	int id;
	/**
	 * @brief The current position of this robot.
	 */
	Coord coord;

	/**
	 * @brief
	 * 3 different robot types DRONE, CATERPILLAR, WHEEL
	 */
	RobotType type = RobotType::INVALID;
	/**
	 * @brief
	 * 3 different robot status IDLE, WORKING, MOVING
	 */
	int status = 3;
	/**
	 * @brief
	 * robot remaining energy
	 */
	int energy = 0;
	/**
	 * @brief
	 * coordinate where robot moving towards
	 */
	Coord targetCoord;

	void move_step() noexcept
	{
		moving_progress_ -= ENERGY_CONSUMPTION_PER_TICK;
		if (moving_progress_ < 0)
			moving_progress_ = 0;

		consume_energy();
	}

	Task &current_task() const
	{
		assert(("Cannot get the current task: the Robot is not working.", current_task_ != nullptr));
		return *current_task_;
	}

	const std::vector<Task> &finished_tasks() const noexcept { return finished_tasks_; }

	/**
	 * @brief Check if this robot reachs a unfinished task.
	 *
	 * @return int Returns the index of the found task.
	 */
	int is_at_task(const std::vector<TaskView> &active_tasks) const noexcept
	{
		for (auto &task_view : active_tasks)
			if (task_view.coord() == coord)
				return task_view.id();
		return -1;
	}

	/**
	 * @brief Set the task object
	 *
	 * @param task
	 */
	void set_task(Task &task)
	{
		assert(static_cast<int>(type) >= 0);
		status = Status::WORKING;
		task_progress_ = task.get_cost_by_type(type);
		current_task_ = &task;
	}

	/**
	 * @brief is robot is working on a task, reduce its task progress by task_progress_per tick
	 * if robot has cosumed all of its energy, robot status becomes EXHAUSTED
	 *
	 * @return true if robot has remaining energy or the task has been completed.
	 * @return false if robot has no remaining energy but task has not been completed.
	 */
	bool do_task() noexcept
	{
		assert(current_task_ != nullptr);

		task_progress_ -= TASK_PROGRESS_PER_TICK;
		if (task_progress_ < 0)
			task_progress_ = 0;

		consume_energy();

		if (status == EXHAUSTED)
		{
			// Exception: exhausted and finished the current task at the same time.
			if (task_progress_ == 0)
			{
				status = WORKING;
				return true;
			}

			return false;
		}
		return true;
	}

	/**
	 * @brief if task is finished, depending on remaining energy, robot status becomes idle or exhasusted
	 *	id of current task is added to finished task list
	 *
	 */
	void finish_task() noexcept
	{
		status = energy == 0 ? EXHAUSTED : IDLE;
		current_task().done = true;
		finished_tasks_.push_back(*current_task_);
		current_task_ = nullptr;
	}

	/**
	 * @brief
	 *
	 */
	void remove_task() noexcept
	{
		current_task_ = nullptr;
	}

	/**
	 * @brief Set the target coordinate object
	 *
	 * @param action
	 * @param verbose
	 */

	void set_target_coordinate(Action action, bool verbose = false)
	{

		static const Coord actions[5] =
			{
				Coord{0, 1},
				Coord{0, -1},
				Coord{-1, 0},
				Coord{1, 0},
				Coord{0, 0}};

		assert(static_cast<int>(action) < 5 && static_cast<int>(action) >= 0);

		targetCoord = coord + actions[static_cast<int>(action)];

		if (targetCoord.x < 0 ||
			targetCoord.y < 0 ||
			targetCoord.x >= MAP_SIZE ||
			targetCoord.y >= MAP_SIZE || action == HOLD)
		{
			// Invalid target.
			status = IDLE;
			targetCoord = coord;
			return;
		}

		else if (object_at(targetCoord) == WALL)
		{
			if (static_cast<int>(action) == 0 || static_cast<int>(action) == 1)
			{
				srand(time(NULL));
				status = IDLE;
				targetCoord = coord + actions[rand() % 2 + 2];
			}

			else if (static_cast<int>(action) == 2 || static_cast<int>(action) == 3)
			{
				srand(time(NULL));
				status = IDLE;
				targetCoord = coord + actions[rand() % 2];
			}
		}

		set_travel_cost();

		status = Status::MOVING;
	}

	/**
	 * @brief returns moving_progress value (see private sections for moving_progress info)
	 *
	 * @return int
	 */
	int
	remaining_moving_progress() const noexcept
	{
		return moving_progress_;
	}

	/**
	 * @brief returns task_progress value (see private sections for task_progress info)
	 *
	 * @return int
	 */
	int remaining_task_progress() const noexcept { return task_progress_; }

	bool is_exhausted() const noexcept { return status == Status::EXHAUSTED; }

	void set_travel_cost() noexcept { moving_progress_ = get_travel_cost() / 2; }

	void reveal_observed_area(std::unordered_map<Coord, Task *> &uncharted_tasks, std::vector<TaskView> &active_tasks)
	{
		if (is_exhausted())
			return;

		if (type == RobotType::DRONE)
		{
			constexpr int DRONE_VIEW_RANGE = 2;
			reveal_square_range(coord, DRONE_VIEW_RANGE, uncharted_tasks, active_tasks);
		}
		else if (type == RobotType::CATERPILLAR)
		{
			constexpr int CATERPILLAR_VIEW_RANGE = 1;
			reveal_square_range(coord, CATERPILLAR_VIEW_RANGE, uncharted_tasks, active_tasks);
		}
		else if (type == RobotType::WHEEL)
		{
			constexpr int WHEEL_VIEW_RANGE = 1;
			reveal_cross_range(coord, WHEEL_VIEW_RANGE, uncharted_tasks, active_tasks);
		}
	}

	/**
	 * @brief Construct a default Robot object
	 */
	Robot()
		: id{-1}, coord{0, 0}, targetCoord{MAP_SIZE + 1, MAP_SIZE + 1}, energy{MAX_ENERGY}, status{IDLE}, type{RobotType::INVALID} {}

	static Robot create_new(int id, Coord position, RobotType type)
	{
		return Robot(id, position, type);
	}

private:
	void consume_energy() noexcept
	{
		energy -= ENERGY_CONSUMPTION_PER_TICK;
		if (energy <= 0)
		{
			energy = 0;
			status = Status::EXHAUSTED;
		}
	}

	int get_travel_cost() const noexcept { return terreinMatrix[type][coord.x][coord.y]; }
	// int get_task_cost() const noexcept { return assignedList[current_working_task_idx].get_cost_by_type(type); }

	Robot(int id, Coord position, RobotType type)
		: id{id}, coord{position}, targetCoord{position}, energy{MAX_ENERGY}, status{IDLE}, type{type} {}

	/*
	 * progress value is used to track how much the moving/task is complete
	 *
	 * moving_progress is set to half of the cost used to pass the current coordinate and when it reaches zero, coordinate of the moving robot changes.
	 *	then, set to half of the cost used to pass this coordinate and moves until it reaches center of the coordinate grid
	 *
	 *  task_progress is set to the taks cost and used task is complete when it reaches zero.
	 */

	int moving_progress_ = 0;
	int task_progress_ = 0;
	Task *current_task_ = nullptr;
	std::vector<Task> finished_tasks_;
};

struct cell
{
	int parent_i, parent_j;
	double f, g, h;
};

struct Scheduler2 : public cell
{
	virtual void on_info_updated(const int (&known_objects)[MAP_SIZE][MAP_SIZE],
								 const int (&known_terrein)[NUM_RTYPE][MAP_SIZE][MAP_SIZE],
								 const std::vector<TaskView> &active_tasks,
								 const Robot (&robot_list)[NUM_ROBOT]) {}

	virtual bool on_task_reached(const int (&known_objects)[MAP_SIZE][MAP_SIZE],
								 const int (&known_terrein)[NUM_RTYPE][MAP_SIZE][MAP_SIZE],
								 const std::vector<TaskView> &active_tasks,
								 const Robot (&robot_list)[NUM_ROBOT],
								 const Task &task,
								 const Robot &current_robot)
	{
		if (current_robot.type == RobotType::DRONE || current_robot.type == RobotType::INVALID)
			return false;
		else
			return true;
	}

	virtual bool isValid(int row, int col) { return true; };

	virtual bool isUnBlocked(int grid[][COL], int row, int col) { return true; };

	virtual bool isDestination(int row, int col, dot dest) { return true; };

	virtual double calculateHValue(int row, int col, dot dest, RobotType robottype) = 0;

	virtual vector<dot> tracePath(cell cellDetails[][COL], dot dest) = 0;

	virtual vector<dot> aStarSearch(int grid[][COL], RobotType robottype, dot src, dot dest) = 0;

	virtual void make_min_min_table(const vector<TaskView> &active_tasks, const Robot (&robots)[NUM_ROBOT]){};

	virtual vector<vector<size_t>> assign_tasks(const vector<TaskView> &active_tasks) = 0;

	virtual void remove_assignment_from_minmin_table(size_t robot_id, size_t task_index){};

	virtual Action calculate_idle_action(const int (&known_objects)[MAP_SIZE][MAP_SIZE],
										 const int (&known_terrein)[NUM_RTYPE][MAP_SIZE][MAP_SIZE],
										 const std::vector<TaskView> &active_tasks,
										 const Robot (&robot_list)[NUM_ROBOT],
										 const Robot &current_robot,
										 int next_x, int next_y) = 0;
};

/**
 * @brief Scheduling algorithms can be applied by modifying functions below.
 * funtion information available above
 */

class Algorithm : public Scheduler2
{

private:
	vector<vector<int>> min_min_table;
	Algorithm *astar;
	Algorithm *trace;

public:
	void on_info_updated(const int (&known_objects)[MAP_SIZE][MAP_SIZE],
						 const int (&known_terrein)[NUM_RTYPE][MAP_SIZE][MAP_SIZE],
						 const std::vector<TaskView> &active_tasks,
						 const Robot (&robot_list)[NUM_ROBOT]) override
	{
	}

	bool on_task_reached(const int (&known_objects)[MAP_SIZE][MAP_SIZE],
						 const int (&known_terrein)[NUM_RTYPE][MAP_SIZE][MAP_SIZE],
						 const std::vector<TaskView> &active_tasks,
						 const Robot (&robot_list)[NUM_ROBOT],
						 const Task &task,
						 const Robot &current_robot) override
	{
		return Scheduler2::on_task_reached(known_objects, known_terrein, active_tasks, robot_list, task, current_robot);
	}

	bool isValid(int row, int col)
	{
		return (row >= 0) && (row <= ROW) && (col >= 0) && (col <= COL);
	}

	bool isUnBlocked(int grid[][COL], int row, int col)
	{
		// Returns true if the cell is not blocked else false
		if (grid[row][col] != WALL)
			return (true);

		if (grid[row][col] == WALL)
			return (false);

		else
		{
			return (true);
		}
	}

	bool isDestination(int row, int col, dot dest)
	{
		if (row == dest.first && col == dest.second)
			return (true);
		else
			return (false);
	}

	double calculateHValue(int row, int col, dot dest, RobotType robottype)
	{
		int x = abs(dest.first - row);
		int y = abs(dest.second - col);
		// Return using the distance formula

		if (robottype == WHEEL)
		{
			return (x + y) * 300; // 150���� �������� ���� �޶���
		}
		if (robottype == CATERPILLAR)
		{
			return (x + y) * 400;
		}
		if (robottype == DRONE)
		{
			return (x + y) * 100;
		}

		return (x + y) * 100;
	}

	// A Utility Function to trace the path from the source
	// to destination
	vector<dot> tracePath(cell cellDetails[][COL], dot dest)
	{
		// printf("\nThe Path is ");
		int row = dest.first;
		int col = dest.second;
		vector<dot> route;

		stack<dot> Path;

		while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col))
		{
			Path.push(make_pair(row, col));
			route.push_back(make_pair(row, col));
			int temp_row = cellDetails[row][col].parent_i;
			int temp_col = cellDetails[row][col].parent_j;
			row = temp_row;
			col = temp_col;
		}

		Path.push(make_pair(row, col));
		route.push_back(make_pair(row, col));
		reverse(route.begin(), route.end());

		// while (!Path.empty())
		// {
		// 	dot p = Path.top();
		// 	Path.pop();
		// 	printf("-> (%d,%d)\n ", p.first, p.second);
		// }

		return route;
	}

	// A Function to find the shortest path between
	// a given source cell to a destination cell according
	// to A* Search Algorithm
	vector<dot> aStarSearch(int grid[][COL], RobotType robottype, dot src, dot dest)
	{
		vector<dot> route;

		// If the source is out of range
		if (isValid(src.first, src.second) == false)
		{
			// printf("Source is invalid\n");
			route.push_back(make_pair(0, 0));
			return route;
		}

		// If the destination is out of range
		if (isValid(dest.first, dest.second) == false)
		{
			// printf("Destination is invalid\n");
			route.push_back(make_pair(0, 0));
			return route;
		}

		// Either the source or the destination is blocked
		if (isUnBlocked(grid, src.first, src.second) == false || isUnBlocked(grid, dest.first, dest.second) == false)
		{
			// printf("Source or the destination is blocked\n");
			route.push_back(make_pair(0, 0));
			return route;
		}

		// If the destination cell is the same as source cell
		if (isDestination(src.first, src.second, dest) == true)
		{
			// printf("We are already at the destination\n");
			route.push_back(make_pair(0, 0));
			return route;
		}

		bool closedList[ROW][COL];
		memset(closedList, false, sizeof(closedList));

		cell cellDetails[ROW][COL];

		int i, j;

		for (i = 0; i < ROW; i++)
		{
			for (j = 0; j < COL; j++)
			{
				cellDetails[i][j].f = FLT_MAX;
				cellDetails[i][j].g = FLT_MAX;
				cellDetails[i][j].h = FLT_MAX;
				cellDetails[i][j].parent_i = -1;
				cellDetails[i][j].parent_j = -1;
			}
		}

		// Initialising the parameters of the starting node
		i = src.first, j = src.second;
		cellDetails[i][j].f = 0.0;
		cellDetails[i][j].g = 0.0;
		cellDetails[i][j].h = 0.0;
		cellDetails[i][j].parent_i = i;
		cellDetails[i][j].parent_j = j;

		set<weight> openList;

		openList.insert(make_pair(0.0, make_pair(i, j)));

		bool foundDest = false;

		// g cost estimation

		int esti, num;

		for (int i = 0; i < MAP_SIZE; i++)
		{
			for (int j = 0; j < MAP_SIZE; j++)
			{
				if (known_terrein_cost_at(robottype, Coord(i, j)) == -1)
				{
					continue;
				}

				else
				{
					esti += known_terrein_cost_at(robottype, Coord(i, j));
					num++;
				}
			}
		}

		esti = esti / num;

		while (!openList.empty())
		{
			weight p = *openList.begin();

			// Remove this vertex from the open list
			openList.erase(openList.begin());

			// Add this vertex to the closed list
			i = p.second.first;
			j = p.second.second;
			closedList[i][j] = true;

			// To store the 'g', 'h' and 'f' of the 4 successors
			double gNew, hNew, fNew;

			//----------- 1st Successor (North) ------------

			// Only process this cell if this is a valid one
			if (isValid(i - 1, j) == true)
			{
				// If the destination cell is the same as the
				// current successor
				if (isDestination(i - 1, j, dest) == true)
				{
					// Set the Parent of the destination cell
					cellDetails[i - 1][j].parent_i = i;
					cellDetails[i - 1][j].parent_j = j;
					// printf("The destination cell is found\n");
					route = tracePath(cellDetails, dest);
					foundDest = true;
					return route;
				}
				// If the successor is already on the closed
				// list or if it is blocked, then ignore it.
				// Else do the following
				else if (closedList[i - 1][j] == false && isUnBlocked(grid, i - 1, j) == true)
				{
					Coord now_value = Coord(i - 1, j);
					if (known_terrein_cost_at(robottype, now_value) == -1)
					{
						gNew = cellDetails[i][j].g + esti;
					}

					else
					{
						gNew = cellDetails[i][j].g + terrein_cost_at(robottype, now_value);
					}

					hNew = calculateHValue(i - 1, j, dest, robottype);
					fNew = gNew + hNew;

					if (cellDetails[i - 1][j].f == FLT_MAX || cellDetails[i - 1][j].f > fNew)
					{
						openList.insert(make_pair(
							fNew, make_pair(i - 1, j)));

						// Update the details of this cell
						cellDetails[i - 1][j].f = fNew;
						cellDetails[i - 1][j].g = gNew;
						cellDetails[i - 1][j].h = hNew;
						cellDetails[i - 1][j].parent_i = i;
						cellDetails[i - 1][j].parent_j = j;
					}
				}
			}

			if (isValid(i + 1, j) == true)
			{

				if (isDestination(i + 1, j, dest) == true)
				{
					cellDetails[i + 1][j].parent_i = i;
					cellDetails[i + 1][j].parent_j = j;
					// printf("The destination cell is found\n");
					route = tracePath(cellDetails, dest);
					foundDest = true;
					return route;
				}

				else if (closedList[i + 1][j] == false && isUnBlocked(grid, i + 1, j) == true)
				{
					Coord now_value = Coord(i + 1, j);
					if (known_terrein_cost_at(robottype, now_value) == -1)
					{
						gNew = cellDetails[i][j].g + esti;
					}

					else
					{
						gNew = cellDetails[i][j].g + terrein_cost_at(robottype, now_value);
					}
					hNew = calculateHValue(i + 1, j, dest, robottype);
					fNew = gNew + hNew;

					if (cellDetails[i + 1][j].f == FLT_MAX || cellDetails[i + 1][j].f > fNew)
					{
						openList.insert(make_pair(
							fNew, make_pair(i + 1, j)));
						// Update the details of this cell
						cellDetails[i + 1][j].f = fNew;
						cellDetails[i + 1][j].g = gNew;
						cellDetails[i + 1][j].h = hNew;
						cellDetails[i + 1][j].parent_i = i;
						cellDetails[i + 1][j].parent_j = j;
					}
				}
			}

			if (isValid(i, j + 1) == true)
			{
				if (isDestination(i, j + 1, dest) == true)
				{
					// Set the Parent of the destination cell
					cellDetails[i][j + 1].parent_i = i;
					cellDetails[i][j + 1].parent_j = j;
					// printf("The destination cell is found\n");
					route = tracePath(cellDetails, dest);
					foundDest = true;
					return route;
				}

				else if (closedList[i][j + 1] == false && isUnBlocked(grid, i, j + 1) == true)
				{
					Coord now_value = Coord(i, j + 1);
					if (known_terrein_cost_at(robottype, now_value) == -1)
					{
						gNew = cellDetails[i][j].g + esti;
					}

					else
					{
						gNew = cellDetails[i][j].g + terrein_cost_at(robottype, now_value);
					}
					hNew = calculateHValue(i, j + 1, dest, robottype);
					fNew = gNew + hNew;

					if (cellDetails[i][j + 1].f == FLT_MAX || cellDetails[i][j + 1].f > fNew)
					{
						openList.insert(make_pair(
							fNew, make_pair(i, j + 1)));

						// Update the details of this cell
						cellDetails[i][j + 1].f = fNew;
						cellDetails[i][j + 1].g = gNew;
						cellDetails[i][j + 1].h = hNew;
						cellDetails[i][j + 1].parent_i = i;
						cellDetails[i][j + 1].parent_j = j;
					}
				}
			}

			if (isValid(i, j - 1) == true)
			{
				if (isDestination(i, j - 1, dest) == true)
				{
					// Set the Parent of the destination cell
					cellDetails[i][j - 1].parent_i = i;
					cellDetails[i][j - 1].parent_j = j;
					// printf("The destination cell is found\n");
					route = tracePath(cellDetails, dest);
					foundDest = true;
					return route;
				}

				else if (closedList[i][j - 1] == false && isUnBlocked(grid, i, j - 1) == true)
				{
					Coord now_value = Coord(i, j - 1);
					if (known_terrein_cost_at(robottype, now_value) == -1)
					{
						gNew = cellDetails[i][j].g + esti;
					}

					else
					{
						gNew = cellDetails[i][j].g + terrein_cost_at(robottype, now_value);
					}
					hNew = calculateHValue(i, j - 1, dest, robottype);
					fNew = gNew + hNew;

					if (cellDetails[i][j - 1].f == FLT_MAX || cellDetails[i][j - 1].f > fNew)
					{
						openList.insert(make_pair(
							fNew, make_pair(i, j - 1)));

						cellDetails[i][j - 1].f = fNew;
						cellDetails[i][j - 1].g = gNew;
						cellDetails[i][j - 1].h = hNew;
						cellDetails[i][j - 1].parent_i = i;
						cellDetails[i][j - 1].parent_j = j;
					}
				}
			}
		}

		if (foundDest == false)
			// printf("Failed to find the Destination Cell\n");

			return route;

		else
			return route;
	}

	void make_min_min_table(const vector<TaskView> &active_tasks, const Robot (&robots)[NUM_ROBOT]) override
	{
		astar = new Algorithm();
		min_min_table.resize(6, vector<int>(active_tasks.size(), 0));

		// Calculate the total cost between each robot and task and store it in the table
		if (active_tasks.size() != 0)
		{
			for (size_t i = 0; i < NUM_ROBOT; i++)
			{
				for (size_t j = 0; j < active_tasks.size(); j++)
				{

					dot a = make_pair(robots[i].coord.x, robots[i].coord.y);
					dot b = make_pair(active_tasks[j].coord().x, active_tasks[j].coord().y);

					int sum = 0;

					if (i == 0 || i == 3)
					{
						min_min_table[i][j] = 0;
					}

					if (i == 1 || i == 4)
					{
						vector<dot> ans = astar->aStarSearch(knownObject, CATERPILLAR, a, b);

						for (int k = 0; k < ans.size(); k++)
						{
							sum += known_terrein_cost_at(CATERPILLAR, Coord(ans[k].first, ans[k].second));
						}

						min_min_table[i][j] = sum;

						if (robots[i].energy < min_min_table[i][j])
						{
							min_min_table[i][j] = 0;
						}

						if (min_min_table[i][j] != 0)
						{

							printf("Robot [%d]의 Task [%d] 수행 비용: %d\n", i, active_tasks[j].id(), min_min_table[i][j]);
						}

						if (min_min_table[i][j] == 0)
						{

							printf("Robot [%d]의 Task [%d] 수행 비용: Robot의 에너지가 부족하여 Task를 수행할 수 없습니다\n", i, active_tasks[j].id());
						}
					}

					if (i == 2 || i == 5)
					{
						vector<dot> ans = astar->aStarSearch(knownObject, WHEEL, a, b);

						for (int k = 0; k < ans.size(); k++)
						{
							sum += known_terrein_cost_at(WHEEL, Coord(ans[k].first, ans[k].second));
						}

						min_min_table[i][j] = sum;

						if (robots[i].energy < min_min_table[i][j])
						{
							min_min_table[i][j] = 0;
						}

						if (min_min_table[i][j] != 0)
						{

							printf("Robot [%d]의 Task [%d] 수행 비용: %d\n", i, active_tasks[j].id(), min_min_table[i][j]);
						}

						if (min_min_table[i][j] == 0)
						{

							printf("Robot [%d]의 Task [%d] 수행 비용: Robot의 에너지가 부족하여 Task를 수행할 수 없습니다\n", i, active_tasks[j].id());
						}
					}
				}
			}
		}
		delete astar;
	}

	// 작업 할당 함수
	vector<vector<size_t>> assign_tasks(const vector<TaskView> &active_tasks)
	{
		// 작업 할당을 위한 변수 초기화
		vector<vector<size_t>> assignments; // 할당된 작업을 저장하는 벡터
		vector<size_t> assignment;
		vector<size_t> min_robot_id(active_tasks.size(), -1);

		size_t robot_index;

		if (active_tasks.size() >= 4)
		{
			// 테이블을 이용하여 작업 할당 결정
			for (size_t i = 0; i < NUM_ROBOT; i++)
			{
				int min_total_cost = INT_MAX;
				unsigned long long min_task_id = 6;

				for (size_t j = 0; j < active_tasks.size(); ++j)
				{
					// 할당되지 않은 작업에 대해서만 검사
					if (min_min_table[i][j] != 0)
					{
						// 최소 비용 갱신
						if (min_min_table[i][j] < min_total_cost)
						{
							min_total_cost = min_min_table[i][j];
							min_task_id = j; // 로봇 i에게 task j를 할당
						}
					}
				}

				// 할당된 작업이 있으면 assignments에 추가
				if (min_task_id != 6)
				{
					printf("Robot [%d]에게 Task [%d]이(가)  할당되었습니다.\n", i, active_tasks[min_task_id].id());
					remove_assignment_from_minmin_table(i, min_task_id);
					vector<size_t> assignment = {i, min_task_id};
					assignments.push_back(assignment);
				}
			}
		}

		if (active_tasks.size() < 4)
		{
			if (active_tasks.size() == 1)
			{
				int min_total_cost = INT_MAX;

				for (size_t i = 0; i < NUM_ROBOT; i++)
				{
					if (min_min_table[i][0] != 0)
					{
						if (min_min_table[i][0] < min_total_cost)
						{
							min_total_cost = min_min_table[i][0];
							min_robot_id[0] = i;
						}
					}
				}

				if (min_robot_id[0] != -1)
				{
					printf("Robot [%d]에게 Task [%d]이(가)  할당되었습니다.\n", min_robot_id[0], active_tasks[0].id());
					remove_assignment_from_minmin_table(min_robot_id[0], 0);
					vector<size_t> assignment = {min_robot_id[0], 0};
					assignments.push_back(assignment);
				}
			}

			if (active_tasks.size() == 2)
			{
				int min_total_cost = INT_MAX;

				for (size_t i = 0; i < NUM_ROBOT; i++)
				{
					if (min_min_table[i][0] != 0)
					{
						if (min_min_table[i][0] < min_total_cost)
						{
							min_total_cost = min_min_table[i][0];
							min_robot_id[0] = i;
						}
					}
				}

				min_total_cost = INT_MAX;

				for (size_t i = 0; i < NUM_ROBOT; i++)
				{
					if (min_min_table[i][1] != 0)
					{
						if (i == min_robot_id[0])
							continue;

						else if (min_min_table[i][1] < min_total_cost)
						{
							min_total_cost = min_min_table[i][1];
							min_robot_id[1] = i;
						}
					}
				}

				for (size_t i = 0; i < 2; i++)
				{
					if (min_robot_id[i] != -1)
					{
						printf("Robot [%d]에게 Task [%d]이(가) 할당되었습니다.\n", min_robot_id[i], active_tasks[i].id());
						remove_assignment_from_minmin_table(min_robot_id[i], i);
						vector<size_t> assignment = {min_robot_id[i], i};
						assignments.push_back(assignment);
					}

					else
						continue;
				}
			}

			if (active_tasks.size() == 3)
			{
				int min_total_cost = INT_MAX;

				for (size_t i = 0; i < NUM_ROBOT; i++)
				{
					if (min_min_table[i][0] != 0)
					{
						if (min_min_table[i][0] < min_total_cost)
						{
							min_total_cost = min_min_table[i][0];
							min_robot_id[0] = i;
						}
					}
				}

				min_total_cost = INT_MAX;

				for (size_t i = 0; i < NUM_ROBOT; i++)
				{
					if (min_min_table[i][1] != 0)
					{
						if (i == min_robot_id[0])
							continue;

						else if (min_min_table[i][1] < min_total_cost)
						{
							min_total_cost = min_min_table[i][1];
							min_robot_id[1] = i;
						}
					}
				}

				min_total_cost = INT_MAX;

				for (size_t i = 0; i < NUM_ROBOT; i++)
				{
					if (min_min_table[i][2] != 0)
					{
						if (i == min_robot_id[0])
							continue;

						if (i == min_robot_id[1])
							continue;

						else if (min_min_table[i][2] < min_total_cost)
						{
							min_total_cost = min_min_table[i][2];
							min_robot_id[2] = i;
						}
					}
				}

				for (size_t i = 0; i < 3; i++)
				{
					if (min_robot_id[i] != -1)
					{
						printf("Robot [%d]에게 Task [%d]이(가) 할당되었습니다.\n", min_robot_id[i], active_tasks[i].id());
						remove_assignment_from_minmin_table(min_robot_id[i], i);
						vector<size_t> assignment = {min_robot_id[i], i};
						assignments.push_back(assignment);
					}

					else
						continue;
				}
			}
		}

		return assignments;
	}

	// Remove the assigned task from the table
	void remove_assignment_from_minmin_table(size_t robot_id, size_t task_index)
	{
		// Set the row and column corresponding to the assigned robot and task to 0
		for (size_t i = 0; i < min_min_table.size(); i++)
		{
			min_min_table[i][task_index] = 0;
		}
		for (size_t i = 0; i < min_min_table[0].size(); i++)
		{
			min_min_table[robot_id][i] = 0;
		}
	}

	Action calculate_idle_action(const int (&known_objects)[MAP_SIZE][MAP_SIZE],
								 const int (&known_terrein)[NUM_RTYPE][MAP_SIZE][MAP_SIZE],
								 const std::vector<TaskView> &active_tasks,
								 const Robot (&robot_list)[NUM_ROBOT],
								 const Robot &current_robot,
								 int next_x, int next_y) override
	{

		if (next_x - current_robot.coord.x > 0)
		{
			return static_cast<Action>(RIGHT);
		}
		else if (next_x - current_robot.coord.x < 0)
		{
			return static_cast<Action>(LEFT);
		}
		else if (next_y - current_robot.coord.y > 0)
		{
			return static_cast<Action>(UP);
		}

		else if (next_y - current_robot.coord.y < 0)
		{
			return static_cast<Action>(DOWN);
		}

		else if (next_x == current_robot.coord.x && next_y == current_robot.coord.y)
		{
			return static_cast<Action>(HOLD);
		}
		else
		{
			return static_cast<Action>(HOLD);
		}
	}
};

int main()
{
	auto time_elapsed = std::chrono::high_resolution_clock::duration::zero();
	auto now = get_now();
	auto update_time_elapsed = [&]()
	{ time_elapsed += (get_now() - now); };

	srand(SEED);
	// Uncomment the line below to get randomised seed.
	// srand(time(NULL));

	Robot robots[NUM_ROBOT];
	Task all_tasks[NUM_MAX_TASKS];
	TaskDispatcher task_dispatcher{};

	/**
	 * @brief A list of currently revealed but not started tasks.
	 */
	std::vector<TaskView> active_tasks;

	std::unordered_map<Coord, Task *> uncharted_tasks;

	/**
	 * The scheduler algorithm for the simulation.
	 * Should change the type of scheulder if you have implemented
	 * algorithms other than `class MyScheduler`.
	 */

	Scheduler2 &algorithm = *new Algorithm();

	vector<vector<size_t>> assignments;
	vector<size_t> assignment;

	// Initialise the map
	generateMap(robots, all_tasks, uncharted_tasks, active_tasks);

	// prints out maps and object informattion
	if (SIMULATOR_VERBOSE)
	{
		printMap(OBJECT);
		printMap(0);
		printMap(1);
		printMap(2);
		printMap(NUMBER);

		printObjects(robots, all_tasks, task_dispatcher.next_task_id());

		std::cout << "Press Enter to start simulation." << std::endl;
		std::getchar();
	}

	// variable to check if all tasks are done
	bool all_done = false;
	int num_working_robots = 0;
	// variable for time stpe
	int time = 0;

	while (time < TIME_MAX && !all_done)
	{
		// checks if task is generate on this time step
		DispatchResult result = task_dispatcher.try_dispatch(time);
		// if task is generated
		if (result.success)
		{
			if (SIMULATOR_VERBOSE)
				std::cout << "Task " << result.task.id << " is generated at " << result.task.coord << ".\n";
			// add task to objectmatrix
			object_at(result.task.coord) = TASK;

			auto id = result.task.id;

			// add task to task list and availabe task list
			all_tasks[id] = std::move(result.task);

			uncharted_tasks.insert({result.task.coord, &all_tasks[id]});

			// Update obseravtions from robots.
			for (Robot &robot : robots)
			{
				robot.reveal_observed_area(uncharted_tasks, active_tasks);
			}
		}

		now = get_now();
		algorithm.on_info_updated(knownObject, knownTerrein, active_tasks, robots);

		if (active_tasks.size() > 0 && time % 50 == 0 && time > 0 && time != TIME_MAX)
		{
			algorithm.make_min_min_table(active_tasks, robots);
			assignments = algorithm.assign_tasks(active_tasks);
		}

		update_time_elapsed();

		int num_exhausted = 0;
		// simulate robot behavior
		for (int index = 0; index < NUM_ROBOT; index++)
		{
			Robot &current_robot = robots[index];

			if (current_robot.status == MOVING) // Robot is currently moving
			{

				// Current robot reaches some point.
				if (current_robot.remaining_moving_progress() == 0)
				{
					if (current_robot.coord == current_robot.targetCoord)
					{
						if (SIMULATOR_VERBOSE)
							std::cout << "Robot " << index << " has reached" << current_robot.coord << std::endl;

						// Check whether current robot reaches a task.
						int task_id = current_robot.is_at_task(active_tasks);

						bool start_task = false;

						if (task_id >= 0) // The current robot reachs task of {task_id}.
						{
							Task &task = all_tasks[task_id];

							// object_at(current_robot.coord) = ROBOT_AND_TASK;
							// num_robots_at(current_robot.coord) += 1;

							// decide wether to start working on the task at the coordinate and add algorithm time
							now = get_now();
							start_task = algorithm.on_task_reached(knownObject, knownTerrein, active_tasks, robots, task, current_robot);
							update_time_elapsed();

							if (start_task)
							{
								current_robot.set_task(task);
								task.assigned_robot_id = current_robot.id;

								// Remove the assigned task's id from the active list.
								auto it = std::find_if(active_tasks.begin(), active_tasks.end(),
													   [task_id](const TaskView &view)
													   { return view.id() == task_id; });
								active_tasks.erase(it);
								num_working_robots++;
							}
						}

						// Current robot has decided to not start working on the reached task.
						// Or, there is no task here.
						if (!start_task)
						{
							current_robot.status = IDLE;
						}
					}
					else // Current robot leaves the current position (coordinate).
					{
						if (SIMULATOR_VERBOSE)
							std::cout << "Robot " << index << " is leaving " << current_robot.coord << std::endl;

						// Make update on object matrix when robotis leaving coordinate
						ObjectType &current_object = object_at(current_robot.coord);

						// Reduce robot count.
						num_robots_at(current_robot.coord) -= 1;
						if (num_robots_at(current_robot.coord) == 0)
						{
							// current_object = current_object == ROBOT
							// 	? EMPTY
							// 	: TASK;
							current_object &= ~(1UL);
						}

						// Move robot to the target coordinate.
						current_robot.coord = current_robot.targetCoord;
						object_at(current_robot.coord) |= ObjectType::ROBOT;
						num_robots_at(current_robot.coord) += 1;

						current_robot.set_travel_cost();

						current_robot.reveal_observed_area(uncharted_tasks, active_tasks);
					}
				}
				// robot has not finished moving yet.
				if (current_robot.remaining_moving_progress() > 0)
				{
					current_robot.move_step();
				}
			}
			else if (current_robot.status == WORKING) // Robot is currently working on the task
			{
				if (current_robot.remaining_task_progress() > 0) // robot has not finished the task yet
				{
					bool success = current_robot.do_task(); // keep working on the task

					if (!success)
					{
						active_tasks.emplace_back(all_tasks[current_robot.current_task().id]);
						current_robot.remove_task();
					}

					if (SIMULATOR_VERBOSE)
						std::cout << "Robot " << index << " is working on task " << current_robot.current_task().id << ".\n";
					if (current_robot.is_exhausted())
					{
						if (SIMULATOR_VERBOSE)
							std::cout << "Robot " << index << " is exhausted while it is working on task " << current_robot.current_task().id << ".\n";
					}
				}
				else // Task done.
				{
					if (SIMULATOR_VERBOSE)
						std::cout << "Robot " << index << " finished working on task " << current_robot.current_task().id << ".\n";

					object_at(current_robot.coord) = ROBOT;
					current_robot.finish_task();
					--num_working_robots;
				}
			}
			else if (current_robot.status == IDLE) // Robot is currently idle
			{
				// decide which direction to move for the robot and add algorithm time
				now = get_now();

				Action action;
				vector<dot> route;
				dot current_coord = make_pair(current_robot.coord.x, current_robot.coord.y);
				int distance1 = robots[0].coord.x;
				int distance2 = robots[3].coord.x;
				int left_robot_id;
				int right_robot_id;

				if (MAP_SIZE <= 30)
				{
					energy_constant = 90;
				}

				if (MAP_SIZE > 30 && MAP_SIZE <= 50)
				{
					energy_constant = 60;
				}

				if (MAP_SIZE > 50)
				{
					energy_constant = 30;
				}

				if (distance_calculate == false)
				{
					if (distance1 < distance2)
					{
						left_robot_id = 0;
						right_robot_id = 3;
						distance_calculate = true;
					}

					if (distance1 >= distance2)
					{
						left_robot_id = 3;
						right_robot_id = 0;
						distance_calculate = true;
					}
				}

				if (current_robot.id == left_robot_id && DRONE1_is_ready == false && distance_calculate == true)
				{

					dot b = make_pair(4 + ii, current_robot.coord.y);
					route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == true)
					{
						if (!route.empty())
						{
							action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
						}

						if (current_robot.coord == Coord(4 + ii, current_robot.coord.y))
						{
							action = static_cast<Action>(HOLD);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
							DRONE1_is_ready = true;
						}
					}

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == false)
					{
						ii++;
					}
				}

				if (current_robot.id == left_robot_id && DRONE1_is_ready == true && DRONE1_step1 == false)
				{

					dot b = make_pair(current_robot.coord.x, MAP_SIZE - 4 - ii);
					route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == true)
					{
						if (!route.empty())
						{
							action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
						}

						if (current_robot.coord == Coord(current_robot.coord.x, MAP_SIZE - 4 - ii))
						{
							action = static_cast<Action>(HOLD);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
							DRONE1_step1 = true;
						}
					}

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == false)
					{
						ii++;
					}
				}

				if (current_robot.id == left_robot_id && DRONE1_step1 == true && DRONE1_step2 == false)
				{

					dot b = make_pair(4 + (7 + (MAP_SIZE / 25)) * (2 * x1 - 1) + ii, current_robot.coord.y);
					route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == true)
					{
						if (!route.empty())
						{
							action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
						}

						if (current_robot.coord == Coord(4 + (7 + (MAP_SIZE / 25)) * (2 * x1 - 1) + ii, current_robot.coord.y))
						{
							action = static_cast<Action>(HOLD);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
							x1++;
							DRONE1_step2 = true;
						}
					}

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == false)
					{
						ii++;
					}
				}

				if (current_robot.id == left_robot_id && DRONE1_step2 == true && DRONE1_step3 == false)
				{

					dot b = make_pair(current_robot.coord.x, 4 + ii);
					route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == true)
					{
						if (!route.empty())
						{
							action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
						}

						if (current_robot.coord == Coord(current_robot.coord.x, 4 + ii))
						{
							action = static_cast<Action>(HOLD);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
							DRONE1_step3 = true;
						}
					}

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == false)
					{
						ii++;
					}
				}

				if (current_robot.id == left_robot_id && DRONE1_step3 == true)
				{

					dot b = make_pair(4 + (7 + (MAP_SIZE / 25)) * 2 * x2 + ii, current_robot.coord.y);
					route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == true)
					{
						if (!route.empty())
						{
							action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
						}

						if (current_robot.coord == Coord(4 + (7 + (MAP_SIZE / 25)) * 2 * x2 + ii, current_robot.coord.y))
						{
							action = static_cast<Action>(HOLD);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
							x2++;
							DRONE1_step1 = false;
							DRONE1_step2 = false;
							DRONE1_step3 = false;
						}
					}

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == false)
					{
						ii++;
					}
				}

				if (current_robot.id == right_robot_id && DRONE2_is_ready == false && distance_calculate == true)
				{

					dot b = make_pair(MAP_SIZE - 4 + ii, current_robot.coord.y);
					route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == true)
					{
						if (!route.empty())
						{
							action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
						}

						if (current_robot.coord == Coord(MAP_SIZE - 4 + ii, current_robot.coord.y))
						{
							action = static_cast<Action>(HOLD);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
							DRONE2_is_ready = true;
						}
					}

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == false)
					{
						ii++;
					}
				}

				if (current_robot.id == right_robot_id && DRONE2_is_ready == true && DRONE2_step1 == false)
				{

					dot b = make_pair(current_robot.coord.x, 4 - ii);
					route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == true)
					{
						if (!route.empty())
						{
							action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
						}

						if (current_robot.coord == Coord(current_robot.coord.x, 4 - ii))
						{
							action = static_cast<Action>(HOLD);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
							DRONE2_step1 = true;
						}
					}

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == false)
					{
						ii++;
					}
				}

				if (current_robot.id == right_robot_id && DRONE2_step1 == true && DRONE2_step2 == false)
				{

					dot b = make_pair(MAP_SIZE - 4 - (7 + (MAP_SIZE / 25)) * (2 * x3 - 1) + ii, current_robot.coord.y);
					route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == true)
					{
						if (!route.empty())
						{
							action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
						}

						if (current_robot.coord == Coord(MAP_SIZE - 4 - (7 + (MAP_SIZE / 25)) * (2 * x3 - 1) + ii, current_robot.coord.y))
						{
							action = static_cast<Action>(HOLD);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
							x3++;
							DRONE2_step2 = true;
						}
					}

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == false)
					{
						ii++;
					}
				}

				if (current_robot.id == right_robot_id && DRONE2_step2 == true && DRONE1_step3 == false)
				{

					dot b = make_pair(current_robot.coord.x, MAP_SIZE - 4 + ii);
					route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == true)
					{
						if (!route.empty())
						{
							action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
						}

						if (current_robot.coord == Coord(current_robot.coord.x, MAP_SIZE - 4 + ii))
						{
							action = static_cast<Action>(HOLD);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
							DRONE2_step3 = true;
						}
					}

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == false)
					{
						ii++;
					}
				}

				if (current_robot.id == right_robot_id && DRONE2_step3 == true)
				{

					dot b = make_pair(MAP_SIZE - 4 - (7 + (MAP_SIZE / 25)) * 2 * x4 + ii, current_robot.coord.y);
					route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == true)
					{
						if (!route.empty())
						{
							action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
						}

						if (current_robot.coord == Coord(MAP_SIZE - 4 - (7 + (MAP_SIZE / 25)) * 2 * x4 + ii, current_robot.coord.y))
						{
							action = static_cast<Action>(HOLD);
							current_robot.set_target_coordinate(action, /*verbose=*/true);
							x4++;
							DRONE2_step1 = false;
							DRONE2_step2 = false;
							DRONE2_step3 = false;
						}
					}

					if (algorithm.isUnBlocked(knownObject, b.first, b.second) == false)
					{
						ii++;
					}
				}

				if (current_robot.type == CATERPILLAR || current_robot.type == WHEEL)
				{
					action = static_cast<Action>(HOLD);
					current_robot.set_target_coordinate(action, /*verbose=*/true);
				}

				if (time > 0 && time != TIME_MAX)
				{
					if (active_tasks.size() != 0 && assignments.size() != 0)
					{
						int a = assignments[0][0];

						if (current_robot.id == a)
						{
							if (assignments[0][1] >= 0 && assignments[0][1] < 16)
							{
								// instance value
								dot b = make_pair(active_tasks[assignments[0][1]].coord().x, active_tasks[assignments[0][1]].coord().y);
								route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

								if (!route.empty())
								{
									action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
									current_robot.set_target_coordinate(action, /*verbose=*/true);
								}

								if (current_robot.coord == Coord(active_tasks[assignments[0][1]].coord().x, active_tasks[assignments[0][1]].coord().y))
								{
									action = static_cast<Action>(HOLD);
									current_robot.set_target_coordinate(action, /*verbose=*/true);
								}
							}
						}

						if (current_robot.id != a && current_robot.type != DRONE)
						{
							if (current_robot.type == WHEEL || current_robot.type == CATERPILLAR)
							{
								if (current_robot.id == 1)
								{
									if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
									{
										if (current_robot.coord.x < (MAP_SIZE / 4) - 2)
										{
											action = static_cast<Action>(RIGHT);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.x > (MAP_SIZE / 4) + 2)
										{
											action = static_cast<Action>(LEFT);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.y > MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE / 4) - 2 && current_robot.coord.x <= (MAP_SIZE / 4) + 2)
										{
											action = static_cast<Action>(DOWN);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.y <= MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE / 4) - 2 && current_robot.coord.x <= (MAP_SIZE / 4) + 2)
										{
											action = static_cast<Action>(HOLD);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}
									}
								}

								if (current_robot.id == 4)
								{
									if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
									{
										if (current_robot.coord.x < (MAP_SIZE * 3 / 4) - 2)
										{
											action = static_cast<Action>(RIGHT);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.x > (MAP_SIZE * 3 / 4) + 2)
										{
											action = static_cast<Action>(LEFT);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.y < (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 2 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) + 2)
										{
											action = static_cast<Action>(UP);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.y >= (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 2 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) + 2)
										{
											action = static_cast<Action>(HOLD);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}
									}
								}

								if (current_robot.id == 2)
								{
									if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
									{
										if (current_robot.coord.x < (MAP_SIZE / 4) + 5)
										{
											action = static_cast<Action>(RIGHT);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.x > (MAP_SIZE / 4) + 9)
										{
											action = static_cast<Action>(LEFT);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.y < (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE / 4) + 5 && current_robot.coord.x <= (MAP_SIZE / 4) + 9)
										{
											action = static_cast<Action>(UP);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.y >= (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE / 4) + 5 && current_robot.coord.x <= (MAP_SIZE / 4) + 9)
										{
											action = static_cast<Action>(HOLD);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}
									}
								}

								if (current_robot.id == 5)
								{
									if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
									{
										if (current_robot.coord.x < (MAP_SIZE * 3 / 4) - 5)
										{
											action = static_cast<Action>(RIGHT);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.x > (MAP_SIZE * 3 / 4) - 9)
										{
											action = static_cast<Action>(LEFT);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.y > MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 5 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 9)
										{
											action = static_cast<Action>(DOWN);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}

										if (current_robot.coord.y <= MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 5 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) - 9)
										{
											action = static_cast<Action>(HOLD);
											current_robot.set_target_coordinate(action, /*verbose=*/true);
										}
									}
								}
							}
						}

						if (assignments.size() >= 2)
						{
							int b = assignments[1][0];

							if (current_robot.id == b)
							{
								if (assignments[1][1] >= 0 && assignments[1][1] < 16)
								{
									// instance value
									dot b = make_pair(active_tasks[assignments[1][1]].coord().x, active_tasks[assignments[1][1]].coord().y);
									route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

									if (!route.empty())
									{
										action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord == Coord(active_tasks[assignments[1][1]].coord().x, active_tasks[assignments[1][1]].coord().y))
									{
										action = static_cast<Action>(HOLD);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}
								}
							}

							if (current_robot.id != a && current_robot.type != DRONE)
							{
								if (current_robot.type == WHEEL || current_robot.type == CATERPILLAR)
								{
									if (current_robot.id == 1)
									{
										if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
										{
											if (current_robot.coord.x < (MAP_SIZE / 4) - 2)
											{
												action = static_cast<Action>(RIGHT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.x > (MAP_SIZE / 4) + 2)
											{
												action = static_cast<Action>(LEFT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y > MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE / 4) - 2 && current_robot.coord.x <= (MAP_SIZE / 4) + 2)
											{
												action = static_cast<Action>(DOWN);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y <= MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE / 4) - 2 && current_robot.coord.x <= (MAP_SIZE / 4) + 2)
											{
												action = static_cast<Action>(HOLD);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}
										}
									}

									if (current_robot.id == 4)
									{
										if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
										{
											if (current_robot.coord.x < (MAP_SIZE * 3 / 4) - 2)
											{
												action = static_cast<Action>(RIGHT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.x > (MAP_SIZE * 3 / 4) + 2)
											{
												action = static_cast<Action>(LEFT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y < (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 2 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) + 2)
											{
												action = static_cast<Action>(UP);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y >= (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 2 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) + 2)
											{
												action = static_cast<Action>(HOLD);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}
										}
									}

									if (current_robot.id == 2)
									{
										if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
										{
											if (current_robot.coord.x < (MAP_SIZE / 4) + 5)
											{
												action = static_cast<Action>(RIGHT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.x > (MAP_SIZE / 4) + 9)
											{
												action = static_cast<Action>(LEFT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y < (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE / 4) + 5 && current_robot.coord.x <= (MAP_SIZE / 4) + 9)
											{
												action = static_cast<Action>(UP);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y >= (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE / 4) + 5 && current_robot.coord.x <= (MAP_SIZE / 4) + 9)
											{
												action = static_cast<Action>(HOLD);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}
										}
									}

									if (current_robot.id == 5)
									{
										if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
										{
											if (current_robot.coord.x < (MAP_SIZE * 3 / 4) - 5)
											{
												action = static_cast<Action>(RIGHT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.x > (MAP_SIZE * 3 / 4) - 9)
											{
												action = static_cast<Action>(LEFT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y > MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 5 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 9)
											{
												action = static_cast<Action>(DOWN);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y <= MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 5 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) - 9)
											{
												action = static_cast<Action>(HOLD);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}
										}
									}
								}
							}
						}

						if (assignments.size() >= 3)
						{
							int c = assignments[2][0];

							if (current_robot.id == c)
							{
								if (assignments[2][1] >= 0 && assignments[2][1] < 16)
								{
									// instance value
									dot b = make_pair(active_tasks[assignments[2][1]].coord().x, active_tasks[assignments[2][1]].coord().y);
									route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

									if (!route.empty())
									{
										action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord == Coord(active_tasks[assignments[2][1]].coord().x, active_tasks[assignments[2][1]].coord().y))
									{
										action = static_cast<Action>(HOLD);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}
								}
							}

							if (current_robot.id != a && current_robot.type != DRONE)
							{
								if (current_robot.type == WHEEL || current_robot.type == CATERPILLAR)
								{
									if (current_robot.id == 1)
									{
										if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
										{
											if (current_robot.coord.x < (MAP_SIZE / 4) - 2)
											{
												action = static_cast<Action>(RIGHT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.x > (MAP_SIZE / 4) + 2)
											{
												action = static_cast<Action>(LEFT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y > MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE / 4) - 2 && current_robot.coord.x <= (MAP_SIZE / 4) + 2)
											{
												action = static_cast<Action>(DOWN);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y <= MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE / 4) - 2 && current_robot.coord.x <= (MAP_SIZE / 4) + 2)
											{
												action = static_cast<Action>(HOLD);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}
										}
									}

									if (current_robot.id == 4)
									{
										if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
										{
											if (current_robot.coord.x < (MAP_SIZE * 3 / 4) - 2)
											{
												action = static_cast<Action>(RIGHT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.x > (MAP_SIZE * 3 / 4) + 2)
											{
												action = static_cast<Action>(LEFT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y < (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 2 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) + 2)
											{
												action = static_cast<Action>(UP);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y >= (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 2 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) + 2)
											{
												action = static_cast<Action>(HOLD);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}
										}
									}

									if (current_robot.id == 2)
									{
										if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
										{
											if (current_robot.coord.x < (MAP_SIZE / 4) + 5)
											{
												action = static_cast<Action>(RIGHT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.x > (MAP_SIZE / 4) + 9)
											{
												action = static_cast<Action>(LEFT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y < (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE / 4) + 5 && current_robot.coord.x <= (MAP_SIZE / 4) + 9)
											{
												action = static_cast<Action>(UP);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y >= (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE / 4) + 5 && current_robot.coord.x <= (MAP_SIZE / 4) + 9)
											{
												action = static_cast<Action>(HOLD);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}
										}
									}

									if (current_robot.id == 5)
									{
										if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
										{
											if (current_robot.coord.x < (MAP_SIZE * 3 / 4) - 5)
											{
												action = static_cast<Action>(RIGHT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.x > (MAP_SIZE * 3 / 4) - 9)
											{
												action = static_cast<Action>(LEFT);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y > MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 5 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 9)
											{
												action = static_cast<Action>(DOWN);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}

											if (current_robot.coord.y <= MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 5 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) - 9)
											{
												action = static_cast<Action>(HOLD);
												current_robot.set_target_coordinate(action, /*verbose=*/true);
											}
										}
									}
								}
							}
						}

						if (assignments.size() >= 4)
						{

							int d = assignments[3][0];
							if (current_robot.id == d)
							{
								if (assignments[3][1] >= 0 && assignments[3][1] < 16)
								{
									// instance value
									dot b = make_pair(active_tasks[assignments[3][1]].coord().x, active_tasks[assignments[3][1]].coord().y);
									route = algorithm.aStarSearch(knownObject, current_robot.type, current_coord, b);

									if (!route.empty())
									{
										action = algorithm.calculate_idle_action(knownObject, knownTerrein, active_tasks, robots, current_robot, route[1].first, route[1].second);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord == Coord(active_tasks[assignments[3][1]].coord().x, active_tasks[assignments[3][1]].coord().y))
									{
										action = static_cast<Action>(HOLD);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}
								}
							}
						}
					}

					if (assignments.size() == 0)
					{
						if (current_robot.type == WHEEL || current_robot.type == CATERPILLAR)
						{
							if (current_robot.id == 1)
							{
								if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
								{
									if (current_robot.coord.x < (MAP_SIZE / 4) - 2)
									{
										action = static_cast<Action>(RIGHT);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.x > (MAP_SIZE / 4) + 2)
									{
										action = static_cast<Action>(LEFT);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.y > MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE / 4) - 2 && current_robot.coord.x <= (MAP_SIZE / 4) + 2)
									{
										action = static_cast<Action>(DOWN);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.y <= MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE / 4) - 2 && current_robot.coord.x <= (MAP_SIZE / 4) + 2)
									{
										action = static_cast<Action>(HOLD);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}
								}
							}

							if (current_robot.id == 4)
							{
								if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
								{
									if (current_robot.coord.x < (MAP_SIZE * 3 / 4) - 2)
									{
										action = static_cast<Action>(RIGHT);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.x > (MAP_SIZE * 3 / 4) + 2)
									{
										action = static_cast<Action>(LEFT);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.y < (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 2 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) + 2)
									{
										action = static_cast<Action>(UP);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.y >= (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 2 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) + 2)
									{
										action = static_cast<Action>(HOLD);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}
								}
							}

							if (current_robot.id == 2)
							{
								if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
								{
									if (current_robot.coord.x < (MAP_SIZE / 4) + 5)
									{
										action = static_cast<Action>(RIGHT);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.x > (MAP_SIZE / 4) + 9)
									{
										action = static_cast<Action>(LEFT);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.y < (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE / 4) + 5 && current_robot.coord.x <= (MAP_SIZE / 4) + 9)
									{
										action = static_cast<Action>(UP);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.y >= (MAP_SIZE * 3) / 4 && current_robot.coord.x >= (MAP_SIZE / 4) + 5 && current_robot.coord.x <= (MAP_SIZE / 4) + 9)
									{
										action = static_cast<Action>(HOLD);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}
								}
							}

							if (current_robot.id == 5)
							{
								if (current_robot.energy >= (MAX_ENERGY * (energy_constant)) / 100)
								{
									if (current_robot.coord.x < (MAP_SIZE * 3 / 4) - 5)
									{
										action = static_cast<Action>(RIGHT);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.x > (MAP_SIZE * 3 / 4) - 9)
									{
										action = static_cast<Action>(LEFT);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.y > MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 5 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 9)
									{
										action = static_cast<Action>(DOWN);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}

									if (current_robot.coord.y <= MAP_SIZE / 4 && current_robot.coord.x >= (MAP_SIZE * 3 / 4) - 5 && current_robot.coord.x <= (MAP_SIZE * 3 / 4) - 9)
									{
										action = static_cast<Action>(HOLD);
										current_robot.set_target_coordinate(action, /*verbose=*/true);
									}
								}
							}
						}
					}
				}

				update_time_elapsed();

				current_robot.set_target_coordinate(action, /*verbose=*/true);
				if (SIMULATOR_VERBOSE && current_robot.targetCoord != current_robot.coord)
					std::cout << "Robot " << index << " targets " << current_robot.targetCoord << ".\n";
			}

			else if (current_robot.status == EXHAUSTED) // Robot has no remaining energy
			{
				// Do nothing for an exhausted robot. (i.e. energy == 0.)
				++num_exhausted;
			}

			else
			{
				std::cout << "Robot " << index << "status error, end simulation." << std::endl;
				std::terminate();
			}
		}

		// Stop simulation if there is no available robot.
		if (num_exhausted == NUM_ROBOT)
			break;

		++time;
		// End condition.
		all_done = active_tasks.empty() &&
				   uncharted_tasks.empty() &&
				   task_dispatcher.num_remaining_tasks() == 0 &&
				   num_working_robots == 0;
	}

	printMap(2);

	if (all_done)
		std::cout << "Finished all tasks at time " << time << ".\n";

	// TODO Report
	for (int index = 0; index < NUM_ROBOT; index++)
		std::cout << "robot " << index << " at (" << robots[index].coord << ") status : " << status_strs[robots[index].status] << " remaining energy: " << robots[index].energy << std::endl;

	std::cout << "Task status: " << std::endl;
	for (auto &task : all_tasks)
	{
		std::cout << "\t- " << task << std::endl;
	}

	std::cout << "Time elapsed running algorithm: " << std::chrono::duration_cast<std::chrono::milliseconds>(time_elapsed).count() << " ms" << std::endl;

	return 0;
}

#pragma region details

void generateMap(Robot *robots, Task *all_tasks, std::unordered_map<Coord, Task *> &uncharted_tasks, std::vector<TaskView> &active_tasks)
{

	// generate walls
	int temp = 0;
	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			temp = rand() % 100;
			if (temp < WALL_DENSITY)
			{
				objectMatrix[ii][jj] = WALL;
			}
		}
	}

	// generate robots
	for (int ii = 0; ii < NUM_ROBOT; ii++)
	{
		Coord position = get_random_empty_position();
		RobotType type = static_cast<RobotType>(ii % NUM_RTYPE);

		robots[ii] = Robot::create_new(ii, position, type);
		object_at(position) = ROBOT;
		num_robots_at(position) += 1;
	}

	// generate initial tasks
	uncharted_tasks.reserve(NUM_MAX_TASKS);
	for (int ii = 0; ii < NUM_INITIAL_TASKS; ii++)
	{
		all_tasks[ii] = Task::generate_random(ii);
		object_at(all_tasks[ii].coord) = TASK;
		uncharted_tasks.insert({all_tasks[ii].coord, &all_tasks[ii]});
	}

	// generate terrein
	int droneCost = (rand() % 40 + 60) * 2;
	int tempCost = 0;
	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			if (objectMatrix[ii][jj] == WALL)
			{
				terreinMatrix[0][ii][jj] = 10;
				terreinMatrix[1][ii][jj] = 10;
				terreinMatrix[2][ii][jj] = 10;
			}
			else
			{
				terreinMatrix[0][ii][jj] = droneCost;
				tempCost = (rand() % 200);
				terreinMatrix[1][ii][jj] = tempCost * 2 + 100;
				terreinMatrix[2][ii][jj] = tempCost * 4 + 50;
			}
		}
	}

	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			for (auto type : robot_types)
				known_terrein_cost_at(type, {ii, jj}) = -1;

			known_object_at({ii, jj}) = UNKNOWN;
		}
	}

	for (int i = 0; i < NUM_ROBOT; ++i)
		robots[i].reveal_observed_area(uncharted_tasks, active_tasks);
}

/*
 *  prints out maps
 *	paramOBJECT
 *	OBJECT : object map
 *	ROBOT TYPE : terrein map for that robot type
 *	NUMBER : map that shows number of robots on that coordinate
 *
 */
void printMap(int type)
{
	if (type == OBJECT)
	{
		printf("Object Map\n\n");
	}
	else if (type == NUMBER)
	{
		printf("Robot count map\n\n");
	}
	else

	{
		printf("Terrein Map for Robot type %d\n\n", type);
	}

	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			if (type == OBJECT)
			{
				switch (objectMatrix[jj][ii])
				{

				case EMPTY:
					printf("   ");
					break;
				case WALL:
					printf("WAL");
					break;
				case ROBOT:
					printf("ROB");
					break;
				case TASK:
					printf("TAS");
					break;
				case ROBOT_AND_TASK:
					printf("RTR");
					break;
				default:
					printf("err");
					break;
				}
			}
			else if (type == NUMBER)
			{
				if (objectMatrix[jj][ii] == WALL)
				{
					printf("WAL");
				}
				else
				{
					printf("%3d", numRobotMatrix[jj][ii]);
				}
			}
			else
			{
				if (knownObject[jj][ii] == WALL)
				{
					printf("WAL");
				}
				else
				{
					printf("%3d", knownTerrein[type][jj][ii]);
				}
			}
			if (jj < MAP_SIZE - 1)
			{
				printf("|");
			}
		}
		printf("\n");
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			if (ii < MAP_SIZE - 1)
			{
				printf("---");
				if (jj < MAP_SIZE - 1)
				{
					printf("o");
				}
			}
		}
		printf("\n");
	}
	printf("\n");
}

// prints out robots and tasks info
void printObjects(const Robot *robotList, const Task *all_tasks, int num_tasks)
{
	std::cout << "##### Robot Locations #####\n";
	for (int ii = 0; ii < NUM_ROBOT; ii++)
		std::cout << "Robot " << ii
				  << "(" << robot_names[static_cast<int>(robotList[ii].type)]
				  << ") at " << robotList[ii].coord << std::endl;
	std::cout << std::endl;

	std::cout << "##### Tasks #####\n";
	for (int i = 0; i < num_tasks; ++i)
	{
		auto &task = all_tasks[i];

		std::cout << "Task " << std::setw(task_num_length) << task.id
				  << " at " << all_tasks[i].coord << " "
				  << "costs: {"
				  << task.get_cost_by_type(RobotType::DRONE) << ", "
				  << task.get_cost_by_type(RobotType::CATERPILLAR) << ", "
				  << task.get_cost_by_type(RobotType::WHEEL) << "}"
				  << (task.done ? " (Done)" : "")
				  << std::endl;
	}
	std::cout << std::endl;

	// for (auto type : robot_types)
	// {
	// 	std::cout << "Type " << robot_names[static_cast<int>(type)];
	// 	for (auto& task_view : active_tasks)
	// 		std::cout << all_tasks[task_view.id()].get_cost_by_type(type) << " \t";
	// 	std::cout << std::endl;
	// }
	// std::cout << std::endl;
}

Coord get_random_empty_position()
{
	// TODO: check if there is any empty place in the map.

	bool valid = false;
	Coord position;
	while (!valid)
	{
		srand(time(NULL));
		position = {rand() % MAP_SIZE, rand() % MAP_SIZE};
		valid = object_at(position) == ObjectType::EMPTY;
	}

	return position;
}

void reveal_square_range(Coord centre, int view_range, std::unordered_map<Coord, Task *> &uncharted_tasks, std::vector<TaskView> &active_tasks)
{
	const int x_min = std::max(0, centre.x - view_range);
	const int x_max = std::min(MAP_SIZE - 1, centre.x + view_range);
	const int y_min = std::max(0, centre.y - view_range);
	const int y_max = std::min(MAP_SIZE - 1, centre.y + view_range);

	for (int x = x_min; x <= x_max; ++x)
		for (int y = y_min; y <= y_max; ++y)
		{
			const Coord c{x, y};

			known_object_at(c) = object_at(c);
			for (RobotType type : robot_types)
				known_terrein_cost_at(type, c) = terrein_cost_at(type, c);

			if (object_at(c) == ObjectType::TASK)
			{
				auto it = uncharted_tasks.find(c);
				if (it != uncharted_tasks.end())
				{
					active_tasks.emplace_back(*it->second);
					uncharted_tasks.erase(it);
				}
			}
		}
}

void reveal_cross_range(Coord centre, int view_range, std::unordered_map<Coord, Task *> &uncharted_tasks, std::vector<TaskView> &active_tasks)
{
	const int x_min = std::max(0, centre.x - view_range);
	const int x_max = std::min(MAP_SIZE - 1, centre.x + view_range);
	const int y_min = std::max(0, centre.y - view_range);
	const int y_max = std::min(MAP_SIZE - 1, centre.y + view_range);

	for (int x = x_min; x <= x_max; ++x)
	{
		const Coord c{x, centre.y};

		known_object_at(c) = object_at(c);
		for (RobotType type : robot_types)
			known_terrein_cost_at(type, c) = terrein_cost_at(type, c);
		if (object_at(c) == ObjectType::TASK)
		{
			auto it = uncharted_tasks.find(c);
			if (it != uncharted_tasks.end())
			{
				active_tasks.emplace_back(*it->second);
				uncharted_tasks.erase(it);
			}
		}
	}

	for (int y = y_min; y <= y_max; ++y)
	{
		const Coord c{centre.x, y};

		known_object_at(c) = object_at(c);
		for (RobotType type : robot_types)
			known_terrein_cost_at(type, c) = terrein_cost_at(type, c);

		if (object_at(c) == ObjectType::TASK)
		{
			auto it = uncharted_tasks.find(c);
			if (it != uncharted_tasks.end())
			{
				active_tasks.emplace_back(*it->second);
				uncharted_tasks.erase(it);
			}
		}
	}
};

#pragma endregion A *
