function mobile_rrr_controller()
  % Combined Mobile Robot + RRR Manipulator Controller
  wb_robot_init();
  TIME_STEP = wb_robot_get_basic_time_step();  % ms
  dt = TIME_STEP/1000;                         % s

  %% === Task 1: DEVICE SETUP ===
  % Mobile base wheels (4-wheel diff drive) 
  wheel_names = ["wheel1","wheel2","wheel3","wheel4"];
  for i=1:4
    wheels(i) = wb_robot_get_device(char(wheel_names(i)));
    assert(wheels(i)~=0, sprintf('Missing device %s',wheel_names(i)));
    wb_motor_set_position(wheels(i), inf);
    wb_motor_set_velocity(wheels(i), 0.0);
  end

  % Distance sensors 
  ds_names = ["right sensor","left_sensor","mid_sensor"];
  for i=1:3
    ds(i) = wb_robot_get_device(char(ds_names(i)));
    assert(ds(i)~=0, sprintf('Missing device %s',ds_names(i)));
    wb_distance_sensor_enable(ds(i), TIME_STEP);
  end
  OBSTACLE_THRESHOLD = 950;  % raw reading < this  obstacle

  % GPS for navigation 
  gps = wb_robot_get_device('gps');
  assert(gps~=0,'Missing GPS');
  wb_gps_enable(gps, TIME_STEP);

  % Pioneer3 gripper (Task 2 choreography) 
  liftMotor   = wb_robot_get_device('lift motor');
  fingerL     = wb_robot_get_device('finger motor::left');
  fingerR     = wb_robot_get_device('finger motor::right');
  assert(liftMotor && fingerL && fingerR, 'Missing gripper motor');
  wb_motor_set_velocity(liftMotor,   0);
  wb_motor_set_velocity(fingerL,      0);
  wb_motor_set_velocity(fingerR,      0);
  GRIPPER_SPEED = 0.1;

  % RRR manipulator (Task 4) 
  % Joint motors & position sensors
  jnames = {'shoulder_motor','elbow motor','wrist motor','rotational motor'};
  for i=1:4
    jm(i) = wb_robot_get_device(jnames{i});
    js(i) = wb_robot_get_device([jnames{i},' sensor']);
    assert(jm(i)~=0 && js(i)~=0, sprintf('Missing %s or its sensor',jnames{i}));
    wb_position_sensor_enable(js(i), TIME_STEP);
    wb_motor_set_position(jm(i), inf);
    wb_motor_set_velocity(jm(i), 0);
  end
  % PID gains
  Kp = [5,5,5,5]; Ki = [1,1,1,1]; Kd = [0.1,0.1,0.1,0.1];
  prev_e = zeros(1,4);
  int_e  = zeros(1,4);

  % Path planning grid (Task 3) 
  GRID_RES = 0.05;    % 5 cm cells
  % define arena bounds in meters (set these to match your world)
  XMIN=0; XMAX=4; YMIN=0; YMAX=3;  
  NX = round((XMAX-XMIN)/GRID_RES);
  NY = round((YMAX-YMIN)/GRID_RES);
  occupancy = zeros(NX,NY);  % 0=free, 1=obstacle
  start_px = 0.2;  start_py = 0.2;      % start (could also read from GPS)
  goal_px  = 2.0;  goal_py  = 0.5;      % manipulator workspace region
  % Precompute goal grid cell
  goal_ix = clamp(round((goal_px-XMIN)/GRID_RES),1,NX);
  goal_iy = clamp(round((goal_py-YMIN)/GRID_RES),1,NY);

  % Generate A* once at start (dynamic updates possible too)
  path = astar([start_px,start_py],[goal_px,goal_py]);
  % Convert to waypoints in meters
  waypoints = cell2mat(path')*[GRID_RES GRID_RES] + [XMIN,YMIN];
  % Build 5th-degree polynomials between successive waypoints
  traj = buildPolynomialTrajectory(waypoints, 5);

  %% === MAIN LOOP ===
  t0 = wb_robot_get_time();
  stage = 1;  % 1=navigate, 2=manipulate, 3=done

  while wb_robot_step(TIME_STEP) ~= -1

    t = wb_robot_get_time()-t0;

    % 1) READ SENSORS
    % distance sensors
    for i=1:3
      ds_val(i) = wb_distance_sensor_get_value(ds(i));
    end
    front_obs = ds_val(2)<OBSTACLE_THRESHOLD;
    left_obs  = ds_val(1)<OBSTACLE_THRESHOLD;
    right_obs = ds_val(3)<OBSTACLE_THRESHOLD;
    % GPS
    pos = wb_gps_get_values(gps);
    x = pos(1); y = pos(2);

    % 2) STAGE MACHINE
    switch stage

      case 1  % === NAVIGATION ===
        % a) obstacle avoidance override
        if front_obs || left_obs || right_obs
          left_v  = 0.2*MAX_SPEED; 
          right_v = 0.6*MAX_SPEED;  % curve away
          if left_obs && right_obs
            left_v  = -MAX_SPEED; right_v = -MAX_SPEED;
          end
        else
          % b) follow polynomial trajectory
          [left_v,right_v] = followTrajectory(traj,t);
        end
        % c) check arrival
        if hypot(x-goal_px,y-goal_py)<0.1
          stage = 2;
          t_manip = t;  % handshake timer
        end

      case 2  % === MANIPULATOR PICK & PLACE ===
        % Timebased openloop coordination:
        dtm = t - t_manip;
        % Sequence identical to Task 2 gripper choreography:
        if dtm<2.0
          lift(0.05);
          moveFingers(0.06);
        elseif dtm<2.5
          moveFingers(0.01);
        elseif dtm<3.0
          lift(0.0);
        end
        left_v=0; right_v=0;

        % RRR manipulator PID to pick/place via IK
        q_des = inverseKinematics( x, y, 0.1 );  % e.g. target at z=0.1
        for i=1:4
          q_curr = wb_position_sensor_get_value(js(i));
          e = q_des(i)-q_curr;
          int_e(i) = int_e(i) + e*dt;
          der = (e-prev_e(i))/dt;
          u = Kp(i)*e + Ki(i)*int_e(i) + Kd(i)*der;
          u = max(min(u,1),-1);
          wb_motor_set_velocity(jm(i), u);
          prev_e(i) = e;
        end

        if dtm>4.0
          stage = 3;
        end

      otherwise  % stage 3: stop everything
        left_v = 0; right_v = 0;
    end

    % 3) SEND VELOCITIES
    wb_motor_set_velocity(wheels(1), left_v);
    wb_motor_set_velocity(wheels(3), left_v);
    wb_motor_set_velocity(wheels(2), right_v);
    wb_motor_set_velocity(wheels(4), right_v);

  end

  wb_robot_cleanup();

  %% === NESTED FUNCTIONS ===

  function v = clamp(x,a,b)
    v = min(max(x,a),b);
  end

  function path = astar(start,goal)
    % A* on occupancy grid with simple AIenhanced heuristic (tiebreaker)
    si = round((start(1)-XMIN)/GRID_RES);
    sj = round((start(2)-YMIN)/GRID_RES);
    gi = goal_ix; gj = goal_iy;
    open = containers.Map();
    cameFrom = containers.Map();
    gScore = inf(NX,NY); gScore(si,sj)=0;
    fScore = inf(NX,NY); fScore(si,sj)=heur(si,sj);
    open(key(si,sj)) = fScore(si,sj);
    while ~isempty(open)
      [ci,cj] = best(open);
      if ci==gi && cj==gj, break; end
      remove(open,key(ci,cj));
      for d=[1 0; -1 0; 0 1; 0 -1]'
        ni=ci+d(1); nj=cj+d(2);
        if ni<1||nj<1||ni>NX||nj>NY||occupancy(ni,nj)==1, continue; end
        tentative = gScore(ci,cj)+1;
        if tentative<gScore(ni,nj)
          cameFrom(key(ni,nj)) = key(ci,cj);
          gScore(ni,nj) = tentative;
          fScore(ni,nj) = tentative + heur(ni,nj);
          open(key(ni,nj)) = fScore(ni,nj);
        end
      end
    end
    % Reconstruct
    cur = key(gi,gj);
    path = {};
    while isKey(cameFrom,cur)
      [i,j] = fromKey(cur); path{end+1}= [i,j]; cur=cameFrom(cur);
    end
    path{end+1} = [si,sj];
    path = fliplr(path);
    function h = heur(i,j)
      % Euclidean + small bias towards straight path
      dx = (gi-i); dy=(gj-j);
      h = sqrt(dx^2+dy^2) * (1 + 0.1*rand());
    end
    function k = key(i,j), k = sprintf('%d,%d',i,j); end
    function remove(m,k), if isKey(m,k), remove(m,k); end; end
    function [i,j] = fromKey(k), tmp = sscanf(k,'%d,%d'); i=tmp(1); j=tmp(2); end
    function [bi,bj] = best(m)
      ks = keys(m); vs = cell2mat(values(m));
      [~,idx] = min(vs); bestk = ks{idx};
      tmp = sscanf(bestk,'%d,%d'); bi=tmp(1); bj=tmp(2);
    end
  end

  function traj = buildPolynomialTrajectory(W,P)
    % W: Nx2 waypoints; P: polynomial degree
    % Build piecewise timeparameterized traj
    N = size(W,1)-1;
    T = linspace(0,1,N+1);  % normalized times
    % Fit separate polynomials for x(t),y(t)
    coefx = polyfit(T,W(:,1)',P);
    coefy = polyfit(T,W(:,2)',P);
    traj.coefx=coefx; traj.coefy=coefy; traj.T=T;
  end

    function [vl,vr] = followTrajectory(traj,t)
    % followTrajectory  Evaluate a smoothed polynomial path
    %   traj has fields coefx, coefy, T
    %   t is the elapsed time in seconds

    % normalized time τ ∈ [0,1]
    tau = min(t / traj.T(end), 1);

    % position on curve (unused here, but you could log xt, yt)
    xt = polyval(traj.coefx, tau);
    yt = polyval(traj.coefy, tau);

    % derivatives dx/dτ, dy/dτ
    dx = polyval(polyder(traj.coefx), tau);
    dy = polyval(polyder(traj.coefy), tau);

    % forward speed along path (in m/s, since τ ∝ t)
    v = hypot(dx, dy);

    % path‐tangent angle
    theta = atan2(dy, dx);

    % differential‐drive inverse kinematics
    L = 0.25;  % half the wheel‐base width (m)
    vl = v - L * theta;
    vr = v + L * theta;

    % clamp to allowed robot speed
    vl = clamp(vl, -MAX_SPEED, MAX_SPEED);
    vr = clamp(vr, -MAX_SPEED, MAX_SPEED);
  end

  function q = inverseKinematics(x, y, z)
  % link lengths
  l1 = 0.2;
  l2 = 0.2;
  l3 = 0.1;

  % 1) radial distance in horizontal plane
  r = hypot(x, y);
  
  % 2) solve wrist tilt theta3 from vertical requirement:
  %    z = l3 * sin(theta3)    theta3 = asin(z / l3)
  % clamp to [-1,1] for safety
  s = z / l3;
  s = max(min(s, 1), -1);
  theta3 = asin(s);
  
  % horizontal projection of the last link
  r3h = l3 * cos(theta3);

  % 3) remaining planar reach for the first two links
  rw = r - r3h;
  if rw < 0
    error('Target too close: rw = %.3f < 0', rw);
  end

  % law-of-cosines for elbow angle theta2:
  D = (rw^2 - l1^2 - l2^2) / (2*l1*l2);
  D = max(min(D,1),-1);  % guard numerical drift
  % choose the elbow-down solution; for elbow-up use -acos(D)
  theta2 = acos(D);

  % 4) shoulder angle theta1 from planar geometry:
  %    theta1 = atan2(y,x) - atan2(l2*sin(theta2), l1 + l2*cos(theta2))
  phi = atan2(y, x);
  alpha = atan2(l2*sin(theta2), l1 + l2*cos(theta2));
  theta1 = phi - alpha;

  % pack and return
  q = [theta1; theta2; theta3];
end


  function lift(pos)
    wb_motor_set_velocity(liftMotor,  GRIPPER_SPEED);
    wb_motor_set_position(liftMotor,  pos);
  end

  function moveFingers(pos)
    wb_motor_set_velocity(fingerL,     GRIPPER_SPEED);
    wb_motor_set_velocity(fingerR,     GRIPPER_SPEED);
    wb_motor_set_position(fingerL,    -pos);
    wb_motor_set_position(fingerR,     pos);
  end

end
